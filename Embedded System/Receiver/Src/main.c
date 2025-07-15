#include "stm32f1xx.h"
#include <stdio.h>

#define RC522_CS_Pin   (1 << 12)
#define RC522_RST_Pin (1 << 15)

void delay_ms(uint32_t ms) {
	for (uint32_t i = 0; i < ms; i++) {
		for (volatile uint32_t j = 0; j < 7200; j++);
	}
}

void enable_gpio(){

// SPI_1 mestre --------------------------------------------------------------

	//NSS
	GPIOA->CRL &= 0xFFF0FFFF; // limpa os bits do A4
	GPIOA->CRL |= 0x00020000; // habilita A4 como out push pull

	//SCK
	GPIOA->CRL &= 0xFF0FFFFF; // limpa os bits do A5
	GPIOA->CRL |= 0x00A00000; // habilita A5 como out AF push pull

	// MISO
	GPIOA->CRL &= 0xF0FFFFFF; // limpa os bits do A6
	GPIOA->CRL |= 0x04000000; // habilita A6 como input floating

	//MOSI
	GPIOA->CRL &= 0x0FFFFFFF; // limpa os bits do A7
	GPIOA->CRL |= 0xA0000000; // habilita A7 como out AF push pull

// SPI_2 mestre --------------------------------------------------------------

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

	GPIOC->CRH &= ~(0xF << 28);   // limpa bits de configuração de PC15
	GPIOC->CRH |= (0x2 << 28);    // saída push-pull

	// Coloca PC15 em nível alto (libera o reset)
	GPIOC->ODR |= (1 << 15);

// USART ---------------------------------------------------------------------

	GPIOA->CRH &= ~(0xF << 4);     // Limpa bits de configuração de PA9
	GPIOA->CRH |= (0xB << 4);      // 1011 = AF output push-pull, 50 MHz

	// PA10 (RX) como input floating
	GPIOA->CRH &= ~(0xF << 8);     // Limpa bits de configuração de PA10
	GPIOA->CRH |= (0x4 << 8);      // 0100 = input floating


// LEDS ----------------------------------------------------------------------

	GPIOC->CRH &= 0xFF0FFFFF; //Set C13 as Output 1111 1111 0000 1111 1111 1111 1111 1111
	GPIOC->CRH |= 0x00200000; // 1111 1111 0010 1111 1111 1111 1111 1111


}

void enable_clock(){
	RCC->APB2ENR |= (1 << 2); //Enable GPIOA Clock A
	RCC->APB2ENR |= (1 << 3); //Enable GPIOC Clock B
	RCC->APB2ENR |= (1 << 4); //Enable GPIOC Clock C
	RCC->APB2ENR |= (1 << 12); //Enable Clock SPI_1
	RCC->APB1ENR |= (1 << 14); //Enable Clock SPI_2
	RCC->APB2ENR |= (1 << 14); //Enable usart1
	RCC->CR |= RCC_CR_HSION; // Liga o HSI (já vem ligado por padrão)
	while (!(RCC->CR & RCC_CR_HSIRDY)); // Espera HSI estabilizar
	RCC->CFGR |= RCC_CFGR_SW_HSI;
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

void enable_usart(){
	// Baud Rate: 9600 com HSI (8MHz) → USARTDIV ≈ 52.08 → 52.1 → 0x0201
	USART1->BRR = (52 << 4) | 1;

	USART1->CR1 = 0;
	USART1->CR1 |= (0 << 12);    // M = 0 (8 bits)
	USART1->CR2 &= ~(3 << 12);   // STOP = 00 (1 stop bit)
	USART1->CR1 |= (1 << 3);     // TE = 1 (habilita transmissor)
	USART1->CR1 |= (1 << 13);    // UE = 1 (USART enable)
}

void uart1_write_char(char c) {
    while (!(USART1->SR & (1 << 7))); // Espera TXE (transmit buffer empty)
    USART1->DR = c;
    while (!(USART1->SR & (1 << 6))); // Espera TC (transmission complete)
}

void uart1_write_string(const char *s) {
    while (*s) {
        uart1_write_char(*s++);
    }
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
}

int Lora_receive(SPI_TypeDef *SPIx, GPIO_TypeDef *GPIOx, uint16_t CS_Pin, uint8_t *buffer) {
	// 1. Coloca o módulo em modo de recepção contínua
	write_reg(SPIx, GPIOx, CS_Pin, 0x01, 0x85); // RegOpMode = LoRa | RXCONTINUOUS

	// 2. Verifica se chegou um pacote (RxDone = bit 6 em RegIrqFlags = 0x12)
	if ((read_reg(SPIx, GPIOx, CS_Pin, 0x12) & (1 << 6)) == 0) {
		return 0; // Nenhum pacote recebido
	}

	// 3. Lê o endereço do FIFO onde o pacote começa
	uint8_t fifo_addr = read_reg(SPIx, GPIOx, CS_Pin, 0x10); // RegFifoRxCurrentAddr

	// 4. Define o ponteiro do FIFO para o endereço correto
	write_reg(SPIx, GPIOx, CS_Pin, 0x0D, fifo_addr); // RegFifoAddrPtr

	// 5. Lê o tamanho do payload
	uint8_t len = read_reg(SPIx, GPIOx, CS_Pin, 0x13); // RegRxNbBytes

	// 6. Lê os dados do FIFO
	for (uint8_t i = 0; i < len; i++) {
		buffer[i] = read_reg(SPIx, GPIOx, CS_Pin, 0x00); // RegFifo = 0x00
	}

	// 7. Limpa o RxDone e outros possíveis flags
	write_reg(SPIx, GPIOx, CS_Pin, 0x12, 0xFF); // limpa todos os IRQs

	// 8. Retorna o número de bytes recebidos
	return len;
}

int main(void) {
	enable_clock();
	enable_gpio();
	enable_spi(SPI1);
	enable_usart();
	Lora_init(SPI1,GPIOA,(1 << 4));
	Lora_set_long_range(SPI1, GPIOA, (1 << 4));
	uint8_t data[5];
	GPIOC->ODR |= (1 << 13); // Apaga LED
	while (1) {
		int n = Lora_receive(SPI1, GPIOA, (1 << 4), data);
		if (n > 0) {
			char buffer[64]; // Tamanho suficiente para "UID:" + até 10 bytes * 2 + \r\n + '\0'
			int pos = 0;

			// Prefixo
			pos += sprintf(buffer + pos, "UID:");

			// Concatena cada byte em hexadecimal
			for (int i = 0; i < n; i++) {
				pos += sprintf(buffer + pos, "%02X", data[i]);
			}

			// Nova linha
			sprintf(buffer + pos, "\r\n");

			// Envia tudo de uma vez
			uart1_write_string(buffer);

			// Pisca LED
			GPIOC->ODR &= ~(1 << 13); // LED ON
			delay_ms(200);
			GPIOC->ODR |= (1 << 13);  // LED OFF
		}
		delay_ms(300);
	}
}

