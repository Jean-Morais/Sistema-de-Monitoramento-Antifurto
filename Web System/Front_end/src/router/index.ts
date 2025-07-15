import { createRouter, createWebHistory } from 'vue-router'
import HomeView from '../views/HomeView.vue'
import AlertView from '../views/AlertView.vue' // ✅ importar a nova página
import LoginView from '../views/LoginView.vue'
import GerenciamentoView from '../views/GerenciamentoView.vue'

import { useUserStore } from '@/stores/user'

const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes: [
    {
      path: '/',
      name: 'home',
      component: HomeView,
      meta: {requiresAuth: true}
    },
    {
      path: '/alertas',
      name: 'alertas',
      component: AlertView, // ✅ registrar a rota de alertas
      meta: {requiresAuth: true}
    },
    {
      path: '/login',
      name: 'login',
      component: LoginView,
    },
    {
      path: '/gerenciamento',
      name: 'gerenciamento',
      component: GerenciamentoView,
      meta: {requiresAuth: true}
    },
    {
      path: '/:pathMatch(.*)*', // Rota catch-all para 404
      name: 'NotFound',
      redirect: '/login' // Redireciona para login se a rota não existir ou para 404
    }
  ],
});

// Navigation Guard
router.beforeEach((to, from, next) => {
  const userStore = useUserStore() // Acessa a store dentro do guard

  // 1. Se a rota requer autenticação e o usuário NÃO está logado
  if (to.meta.requiresAuth && !userStore.token) {
    next({ name: 'login' })
  }
  // 2. Se o usuário TENTA ACESSAR A PÁGINA DE LOGIN enquanto JÁ ESTÁ LOGADO
  else if (to.name === 'login' && userStore.token) {
    next({ name: 'home' }) // Redireciona para a home/dashboard
  }
  // 3. Em todos os outros casos (autenticado para rota autenticada, ou não autenticado para rota não autenticada)
  else {
    next()
  }
})

export default router
