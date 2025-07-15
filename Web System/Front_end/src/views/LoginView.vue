<template>
  <div class="login-container">
    <div class="login-header">
      <h1 class="title">Secure<span>Track</span></h1>
      <h2 class="subtitle">Sistema de Monitoramento Anti-Furto</h2>
      <p class="university">🏛️ Universidade Federal</p>
    </div>
    <div class="login-card">
      <form @submit.prevent="handleLogin" class="form">
        <div class="logo_cadeado">
          <img src="@/assets/padlock.svg" alt="cadeado" />
          <h3 class="form-title">Fazer Login</h3>
        </div>
        <p class="form-subtitle">Acesse o sistema de monitoramento</p>

        <label for="email">Email</label>
        <input
          id="email"
          type="email"
          v-model="email"
          placeholder="seu.email@universidade.edu.br"
          required
        />

        <label for="senha">Senha</label>
        <input
          id="senha"
          type="password"
          v-model="password"
          placeholder="Senha"
          required
        />

        <div class="invalidLogin" v-if="invalidLogin">
          {{ invalidText }}
        </div>

        <button path="/dashboard"
        type="submit">Entrar no Sistema</button>

      </form>
    </div>
    <footer class="footer">
      Sistema desenvolvido para proteção do patrimônio universitário<br />
      Monitoramento RFID • Alertas em tempo real • Segurança avançada
    </footer>
  </div>
</template>

<script lang="ts" setup>
import { ref } from 'vue'
import { useRouter } from 'vue-router'
import api from '@/api/api'
import { useUserStore } from '@/stores/user'
import {jwtDecode} from 'jwt-decode'

const invalidLogin = ref(false)
const invalidText = ref('')

const email = ref('')
const password = ref('')
const router = useRouter()
const userStore = useUserStore()

interface DecodedJWT {
  id: number
  email: string
  username: string
}

async function checkLogin(email: string, password: string) {
  const data = {
    email: email,
    password: password
  }

  try {
    const response = await api.post('/auth', data)

    const token = response.data.token

    // Decodifica o token
    const decoded = jwtDecode<DecodedJWT>(token)

    // Salva os dados no store
    userStore.setUserData({
      token,
      id: decoded.id,
      email: decoded.email,
      username: decoded.username
    })

    router.push('/')

  } catch (error: any) {
    invalidText.value = 'Email ou senha inválidos'
    invalidLogin.value = true
    console.error(error);
  }
}

async function handleLogin() {
  await checkLogin(email.value,password.value)
}
</script>

<style scoped>
.login-container {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  min-height: 100vh;
  background: linear-gradient(to top left, #f5f9ff, #eef3f9);
  padding: 2rem;
}
.logo_cadeado {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 0.5rem;
  margin-bottom: 0.5rem;
}

.logo_cadeado img {
  width: 28px;
  height: 28px;
}

body {
  font-family: "Inter", sans-serif;
}

.login-header {
  text-align: center;
  margin-bottom: 2rem;
}

.title {
  font-size: 1.75rem;
  font-weight: 700;
  margin: 0;
}

.title span {
  color: inherit;
}

.subtitle {
  font-size: 1rem;
  color: #444;
  margin: 0.25rem 0;
}

.university {
  font-size: 0.9rem;
  color: #888;
}

.login-card {
  background: white;
  border-radius: 1rem;
  padding: 2rem;
  max-width: 480px;
  width: 100%;
  box-shadow: 0 4px 16px rgba(0, 0, 0, 0.08);
  text-align: center;
  margin-bottom: 2rem;
}

.form {
  display: flex;
  flex-direction: column;
  gap: 0.75rem;
  text-align: left;
}

.form-title {
  font-size: 1.50rem;
  font-weight: 600;
  text-align: center;
}


.form-subtitle {
  font-size: 0.9rem;
  color: #666;
  margin-bottom: 0.8rem;
  text-align: center;
}

label {
  font-size: 0.85rem;
  margin-left: 0.2rem;
  color: #333;
}

input {
  padding: 0.6rem;
  border: 1px solid #ccc;
  border-radius: 0.5rem;
  font-size: 1rem;
}

button {
  padding: 0.6rem;
  background-color: #2563eb;
  color: white;
  border: none;
  border-radius: 0.5rem;
  font-size: 1rem;
  margin-top: 0.8rem;
  cursor: pointer;
  transition: background-color 0.2s;
}

button:hover {
  background-color: #1d4ed8;
}

.footer {
  text-align: center;
  font-size: 0.75rem;
  color: #888;
  line-height: 1.4;
}
</style>
