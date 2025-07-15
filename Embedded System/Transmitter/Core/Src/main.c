#include "main.h"
#include "rc522.h"
#include "string.h"
#include "stm32f1xx.h"

SPI_HandleTypeDef hspi1;

uint8_t status;
uint8_t str[MAX_LEN]; // Max_LEN = 16
uint8_t sNum[5];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

void delay_ms(uint32_t ms) {
	for (uint32_t i = 0; i < ms; i++) {
		for (volatile uint32_t j = 0; j < 7200; j++);
	}
}

void enable_spi(SPI_TypeDef *SPIx){
	SPIx->CR1 = 0; // limpa tudo
	SPIx->CR1 |= (2 << 3); // Define a taxa de transmissão do SCK fsk/8
	SPIx->CR1 &= ~((1 << 1) | (1 << 0));
	//SPI1->CR1 &= ~(1 << 11); // Formato de quadros = 8
	SPIx->CR1 &= ~(1 << 7);
	SPIx->CR1 |= (1 << 9); // Habilita NSS via software
	SPIx->CR1 |= (1 << 8); // Entrada do SSM
	SPIx->CR1 |= (1 << 2); // Mestre
	SPIx->CR1 |= (1 << 6); // Habilita SPI
}

uint8_t spi_transfer(SPI_TypeDef *SPIx, uint8_t data) {
	while (!(SPIx->SR & SPI_SR_TXE));   // Espera buffer de transmissão vazio
	SPIx->DR = data;                    // Envia o dado
	while (!(SPIx->SR & SPI_SR_RXNE));  // Espera dado recebido
	return SPIx->DR;                    // Retorna o dado recebido
}

void write_reg(SPI_TypeDef *SPIx, GPIO_TypeDef *GPIOx, uint16_t CS_Pin, uint8_t addr, uint8_t val) {
	GPIOx->ODR &= ~CS_Pin;          // CS baixo (ativo)
	spi_transfer(SPIx, addr | 0x80);// bit 7 = 1 para escrita
	spi_transfer(SPIx, val);        // dado a escrever
	GPIOx->ODR |= CS_Pin;           // CS alto (inativo)
}

uint8_t read_reg(SPI_TypeDef *SPIx, GPIO_TypeDef *GPIOx, uint16_t CS_Pin, uint8_t addr) {
	uint8_t val;
	GPIOx->ODR &= ~CS_Pin;          // CS baixo
	spi_transfer(SPIx, addr & 0x7F);// bit 7 = 0 para leitura
	val = spi_transfer(SPIx, 0x00); // dummy write para ler dado
	GPIOx->ODR |= CS_Pin;           // CS alto
	return val;
}

void Lora_init(SPI_TypeDef *SPIx, GPIO_TypeDef *GPIOx, uint16_t CS_Pin) {
	// 1. Sleep mode (necessário antes de alterar OpMode)
	write_reg(SPIx, GPIOx, CS_Pin, 0x01, 0x80);  // RegOpMode: Sleep, LoRa
	delay_ms(10);

	// 2. Standby mode
	write_reg(SPIx, GPIOx, CS_Pin, 0x01, 0x81);  // RegOpMode: Standby, LoRa
	delay_ms(10);

	// 3. Frequência: 433 MHz → Freq = (FRF * 32 MHz) / 2^19
	// FRF = (433000000 * 2^19) / 32 MHz = 0x6C8000
	write_reg(SPIx, GPIOx, CS_Pin, 0x06, 0x6C);  // RegFrfMsb
	write_reg(SPIx, GPIOx, CS_Pin, 0x07, 0x80);  // RegFrfMid
	write_reg(SPIx, GPIOx, CS_Pin, 0x08, 0x00);  // RegFrfLsb

	// 4. Potência: PA_BOOST, potência máxima
	write_reg(SPIx, GPIOx, CS_Pin, 0x09, 0x8F);  // RegPaConfig

	// 5. Configuração dos endereços FIFO
	write_reg(SPIx, GPIOx, CS_Pin, 0x0E, 0x80);  // RegFifoTxBaseAddr = 0x80
	write_reg(SPIx, GPIOx, CS_Pin, 0x0F, 0x00);  // RegFifoRxBaseAddr = 0x00

	// 6. Modulação LoRa
	// Largura de banda = 125 kHz, CR = 4/5, modo implícito desligado
	write_reg(SPIx, GPIOx, CS_Pin, 0x1D, 0x72);  // RegModemConfig1: BW125, CR 4/5
	// SF = 7, CRC habilitado
	write_reg(SPIx, GPIOx, CS_Pin, 0x1E, 0x74);  // RegModemConfig2: SF7, CRC on
	// Auto-AGC ligado
	write_reg(SPIx, GPIOx, CS_Pin, 0x26, 0x04);  // RegModemConfig3

	// 7. Mapeamento de pinos: DIO0 = TxDone
	write_reg(SPIx, GPIOx, CS_Pin, 0x40, 0x40);  // RegDioMapping1

	// 8. Modo contínuo de recepção pode ser ativado depois com:
	// write_reg(SPIx, GPIOx, CS_Pin, 0x01, 0x85); // RX continuous

	// 9. Volta ao modo standby pronto para TX/RX
	write_reg(SPIx, GPIOx, CS_Pin, 0x01, 0x81);
}

void Lora_set_long_range(SPI_TypeDef *SPIx, GPIO_TypeDef *GPIOx, uint16_t CS_Pin) {
    // 1. Entrar em modo Sleep (necessário para mudar parâmetros)
    write_reg(SPIx, GPIOx, CS_Pin, 0x01, 0x80);  // RegOpMode: Sleep, LoRa
    delay_ms(10);

    // 2. Largura de banda = 62.5 kHz, CR = 4/5 (BW ↓ → alcance ↑)
    write_reg(SPIx, GPIOx, CS_Pin, 0x1D, 0x62);  // RegModemConfig1: BW62.5kHz, CR4/5

    // 3. SF = 12 (maior espalhamento possível), CRC habilitado
    write_reg(SPIx, GPIOx, CS_Pin, 0x1E, 0xB4);  // RegModemConfig2: SF12, CRC on

    // 4. Otimização para dados lentos (obrigatório para SF11 e SF12)
    write_reg(SPIx, GPIOx, CS_Pin, 0x26, 0x0C);  // RegModemConfig3: AGC auto + LowDataRateOptimize

    // 5. Potência máxima com PA_BOOST (até 20 dBm)
    write_reg(SPIx, GPIOx, CS_Pin, 0x09, 0x8F);  // RegPaConfig: PA_BOOST, Pout = 17dBm
    write_reg(SPIx, GPIOx, CS_Pin, 0x4D, 0x87);  // RegPaDac: High Power mode → 20 dBm

    // 6. Mapeamento DIO0 para TxDone/RxDone
    write_reg(SPIx, GPIOx, CS_Pin, 0x40, 0x40);  // RegDioMapping1: DIO0 = TxDone (00)

    // 7. Voltar ao modo standby (pronto para enviar ou receber)
    write_reg(SPIx, GPIOx, CS_Pin, 0x01, 0x81);  // RegOpMode: Standby, LoRa
}


void Lora_send(SPI_TypeDef *SPIx, GPIO_TypeDef *GPIOx, uint16_t CS_Pin, const uint8_t *data, uint8_t len) {
	// 1. Coloca endereço de início do FIFO de TX
	write_reg(SPIx, GPIOx, CS_Pin, 0x0D, 0x80); // RegFifoAddrPtr = FifoTxBaseAddr

	// 2. Escreve os dados no FIFO
	for (uint8_t i = 0; i < len; i++) {
		write_reg(SPIx, GPIOx, CS_Pin, 0x00, data[i]); // RegFifo = 0x00
	}

	// 3. Define o comprimento do payload
	write_reg(SPIx, GPIOx, CS_Pin, 0x22, len); // RegPayloadLength

	// 4. Coloca o módulo em modo de transmissão
	write_reg(SPIx, GPIOx, CS_Pin, 0x01, 0x83); // RegOpMode = LoRa | TX

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); // LED ON (PC13 é geralmente invertido)
	delay_ms(300);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // LED OFF
	delay_ms(300);

}

int main(void)
{
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	MX_SPI1_Init();
	MFRC522_Init();
	enable_spi(SPI2);
	Lora_init(SPI2,GPIOB,(1 << 12));
	Lora_set_long_range(SPI2,GPIOB,(1 << 12));

	while (1) {
		status = MFRC522_Request(PICC_REQIDL, str);
		if (status == MI_OK) {
			status = MFRC522_Anticoll(str);
			if (status == MI_OK) {
				Lora_send(SPI2, GPIOB, (1 << 12), str, 5);
				delay_ms(300);
			}
		}
	}
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC->APB2ENR |= (1 << 3); //Enable GPIOC Clock B
  RCC->APB1ENR |= (1 << 14); //Enable Clock SPI_2

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  //NSS
	GPIOB->CRH &= 0xFFF0FFFF; // limpa os bits do B12
	GPIOB->CRH |= 0x00020000; // habilita B12 como out push pull

	//SCK
	GPIOB->CRH &= 0xFF0FFFFF; // limpa os bits do B13
	GPIOB->CRH |= 0x00A00000; // habilita B13 como out AF push pull

	// MISO
	GPIOB->CRH &= 0xF0FFFFFF; // limpa os bits do B14
	GPIOB->CRH |= 0x04000000; // habilita B14 como input floating

	//MOSI
	GPIOB->CRH &= 0x0FFFFFFF; // limpa os bits do B15
	GPIOB->CRH |= 0xA0000000; // habilita B15 como out AF push pull
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
