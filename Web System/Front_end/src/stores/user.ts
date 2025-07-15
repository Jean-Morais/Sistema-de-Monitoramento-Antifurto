import { defineStore } from 'pinia'

interface UserState {
  token: string | null
  id: number | null
  email: string | null
  username: string | null
}

export const useUserStore = defineStore('user', {
  state: (): UserState => ({
    token: null,
    id: null,
    email: null,
    username: null,
  }),

  actions: {
    setUserData({ token, id, email, username }: UserState) {
      this.token = token
      this.id = id
      this.email = email
      this.username = username
    },

    clearUserData() {
      this.token = null
      this.id = null
      this.email = null
      this.username = null
    },
  },
  persist: true // se estiver usando pinia-plugin-persistedstate (opcional)
})
