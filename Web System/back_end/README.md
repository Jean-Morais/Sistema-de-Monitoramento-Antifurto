## **Guia de Instalação e Execução do Projeto**

Este guia detalha os passos necessários para configurar, instalar e executar o projeto em seu ambiente de desenvolvimento.

-----

### **1. Pré-requisitos**

Antes de começar, certifique-se de ter os seguintes softwares instalados em sua máquina:

  * **Node.js:** Versão [Ex: 18.x ou superior]. Você pode verificar sua versão com `node -v`.
      * Recomendamos usar o [NVM (Node Version Manager)](https://github.com/nvm-sh/nvm) para gerenciar as versões do Node.js.
  * **Yarn** ou **npm:** Gerenciador de pacotes. `npm` geralmente vem com o Node.js. Para instalar o Yarn, use `npm install -g yarn`.
  * **Git:** Para clonar o repositório.
  * **Docker** e **Docker Compose** (Opcional, se o projeto usar contêineres para banco de dados ou outros serviços): Para gerenciar serviços em contêineres.

-----

### **2. Configuração do Ambiente**

#### **2.1. Clonar o Repositório**

Primeiro, clone o repositório do projeto para sua máquina local:

```bash
git clone https://github.com/dam1aoGomes/sistema-anti-furto.git
cd sistema-anti-furto
```

#### **2.2. Variáveis de Ambiente**

Este projeto utiliza variáveis de ambiente para configurações sensíveis e específicas do ambiente. Você precisará criar um arquivo `.env` na raiz do projeto.

1.  Crie um arquivo `.env` copiando o arquivo de exemplo:

    ```bash
    cp .env.example .env
    ```

2.  Edite o arquivo `.env` recém-criado e preencha as variáveis conforme necessário. As variáveis mínimas esperadas são:

    ```
    # Configurações do Servidor
    PORT=3000

    # Configurações do Banco de Dados
    DATABASE_URL="postgresql://user:senha@localhost:5432/seubanco"

    # JWT (se aplicável)
    JWT_SECRET=sua_chave_secreta_muito_forte

    # Send Grid Api
    SENDGRID_API_KEY=sua_apikey_send_grid
    ```

    **Importante:** Nunca commite seu arquivo `.env` para o controle de versão\! Ele já está incluído no `.gitignore`.

-----

### **3. Instalação de Dependências**

Após configurar o ambiente, instale as dependências do projeto.

Se estiver usando **Yarn**:

```bash
yarn install
```

Se estiver usando **npm**:

```bash
npm install
```

-----

### **4. Configuração do Banco de Dados**

#### **Banco de Dados Local**

Se você prefere instalar o banco de dados diretamente em sua máquina ou já tem um configurado:

1.  Certifique-se de que o serviço do seu banco de dados (ex: PostgreSQL) esteja rodando.
2.  Crie o banco de dados especificado em seu arquivo `.env` (`DB_NAME`). Por exemplo, para PostgreSQL:
    ```bash
    psql -U seu_usuario_db -c "CREATE DATABASE seu_banco_de_dados;"
    ```

#### **4.1. Rodar Migrações e Seeds (se aplicável)**

Após configurar o banco de dados, você precisará rodar as migrações para criar o esquema e, opcionalmente, as seeds para popular o banco com dados iniciais.

```bash
# Rodar migrações
npx prisma migrate dev
```

-----

### **5. Execução do Projeto**

Agora você pode iniciar o servidor da API.

#### **Modo Desenvolvimento (com Nodemon para hot-reloading)**

```bash
yarn run dev # ou npm run dev
```

O servidor estará disponível em `http://localhost:3000` (ou a porta configurada em seu `.env`).

#### **Modo Produção (Build e Start)**

Para simular o ambiente de produção, primeiro compile o código TypeScript para JavaScript e depois inicie o servidor:

```bash
# Compilar o código TypeScript
yarn run build # ou npm run build

# Iniciar o servidor de produção
yarn run start # ou npm run start
```

-----

### **6. Testes**

Para rodar os testes automatizados do projeto:

```bash
yarn test # ou npm test
```

-----

### **7. Contribuição**

Consulte o arquivo `CONTRIBUTING.md` (se existir) para informações sobre como contribuir para este projeto.

-----

### **9. Solução de Problemas Comuns**

  * **Erro de conexão com o banco de dados:** Verifique suas variáveis `.env` (`DB_HOST`, `DB_PORT`, `DB_USER`, `DB_PASSWORD`, `DB_NAME`) e certifique-se de que o serviço do banco de dados está rodando (seja via Docker ou localmente).
  * **Porta já em uso:** Altere a variável `PORT` em seu arquivo `.env` para uma porta diferente (ex: `3001`).
  * **Dependências não encontradas:** Execute `yarn install` ou `npm install` novamente.
  * **Erros de compilação TypeScript:** Verifique a saída do comando `yarn run dev` (ou `npm run dev`) ou `yarn run build` (ou `npm run build`) para mensagens de erro específicas.

-----

**Parabéns\!** Você configurou e executou o projeto. Se tiver qualquer problema ou dúvida, não hesite em procurar ajuda.
