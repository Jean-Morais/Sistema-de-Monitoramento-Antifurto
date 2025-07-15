
# Sistema de Monitoramento Antifurto

![C](https://img.shields.io/badge/c-%2300599C.svg?style=for-the-badge&logo=c&logoColor=white) 
![NodeJS](https://img.shields.io/badge/node.js-6DA55F?style=for-the-badge&logo=node.js&logoColor=white)
![Express.js](https://img.shields.io/badge/express.js-%23404d59.svg?style=for-the-badge&logo=express&logoColor=%2361DAFB)
![Postgres](https://img.shields.io/badge/postgres-%23316192.svg?style=for-the-badge&logo=postgresql&logoColor=white)
![Vue.js](https://img.shields.io/badge/vuejs-%2335495e.svg?style=for-the-badge&logo=vuedotjs&logoColor=%234FC08D)

## Sobre o sistema

O sistema realiza o monitoramento de entradas e saídas de ambientes, salas, blocos, dentre outros. E, detecta se está acontecendo alguma movimentação indesejada de bens patrimoniais da instituição e realiza o devido alerta aos usuários do sistema.

O projeto é divido em duas vertentes, a parte embarcada foi desenvolvida em bare metal com microcontroladores SMT32F103C8T6, comunicação a distância com módulos LORA, monitoramento com tags e leitores RFIDs e comunicação web com ESP01. A parte web foi desenvolvida com o back end em node.js + express.js, o front end foi em vue.js e o banco de dados em postgres. 

O projeto foi desenvolvido na cadeira de Engenharia de Software para nosso cliente Abdul, gerente do almoxarifado de eletrônicos da Universidade Federal do Ceará campus Quixadá.


