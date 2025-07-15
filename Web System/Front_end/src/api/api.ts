import axios from 'axios'

const api = axios.create({
  baseURL: 'https://sistema-anti-furto.vercel.app/api',
  timeout: 5000,
})

export default api