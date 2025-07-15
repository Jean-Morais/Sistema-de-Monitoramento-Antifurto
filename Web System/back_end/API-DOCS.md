---

# 📘 API Documentation – Sistema Anti-Furto

---

## 🔐 Auth Endpoints

### 🔑 Login

**Endpoint:**
`POST /api/auth`

**Request Body (JSON):**

```json
{
  "email": "email@email.com",
  "password": "PasswordStrong1!"
}
```

**Response:**

```json
{
  "token": "jwt_token_here"
}
```

**Status Code:** `200 OK`

---

### 📝 Register

**Endpoint:**
`POST /api/register`

**Request Body (JSON):**

```json
{
  "name": "Your Name",
  "email": "email@email.com",
  "password": "PasswordStrong1!"
}
```

**Response:**

```json
{
  "id": 0,
  "name": "Your Name",
  "email": "email@email.com"
}
```

**Status Code:** `201 Created`

---

## ⚙️ Equipments Endpoints

> Todas as rotas abaixo requerem autenticação via **Bearer Token (JWT)**.

---

### 🔍 Buscar Equipamentos

**Endpoint:**
`GET /api/equipament`
`GET /api/equipament?query`

**Query Params (opcionais):**

* `whereName=string`
* `type=string`
* `location=string`
* `responsible=string`
* `inRange=true|false`
* `page=1` (padrão: 1)
* `limit=10` (padrão: 10)

**Exemplo de Resposta:**

```json
{
  "data": [
    {
      "id": 10,
      "name": "Sensor de Temperatura",
      "type": "Sensor",
      "location": "Laboratório 2",
      "rfid": "ABC1234567898",
      "responsible": "Carlos Silva",
      "isInRange": true,
      "updatedAt": "2025-07-10T19:25:30.173Z"
    },
    {
      "id": 11,
      "name": "Sensor de Temperatura",
      "type": "Sensor",
      "location": "Laboratório 2",
      "rfid": "ABC1234567899",
      "responsible": "Carlos Silva",
      "isInRange": true,
      "updatedAt": "2025-07-10T19:25:32.736Z"
    }
  ],
  "total": 21,
  "page": 1,
  "limit": 10,
  "totalPages": 3
}
```

**Status Code:** `200 OK`

---

### ➕ Criar Novo Equipamento

**Endpoint:**
`POST /api/equipament`

**Request Body (JSON):**

```json
{
  "name": "Sensor de Temperatura",
  "type": "Sensor",
  "location": "Laboratório 2",
  "rfid": "ABC12345678920",
  "responsible": "Carlos Silva"
}
```

**Response:**

```json
{
  "id": 23,
  "name": "Sensor de Temperatura",
  "type": "Sensor",
  "location": "Laboratório 2",
  "rfid": "ABC12345678920",
  "responsible": "Carlos Silva",
  "isInRange": true,
  "updatedAt": "2025-07-12T02:55:57.367Z"
}
```

**Status Code:** `201 Created`

---

### 🚨 Fazer Alerta (Equipamento Fora de Alcance)

**Endpoint:**
`POST /api/equipament/alert-out-of-range`

**Request Body (JSON):**

```json
{
  "rfid": "CODE123"
}
```

**Response:**
Sem body de resposta.

**Status Code:** `200 OK`

---
