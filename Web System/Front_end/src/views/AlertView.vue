<template>
  <div class="alert-view">
    <!-- Cabeçalho -->
    <header class="top-bar">
      <div class="title-area">
        <h1 class="main-title">Sistema de Alertas</h1>
        <p class="subtitle">Monitoramento e gestão de alertas de segurança</p>
      </div>
      <button @click="voltar" class="btn-back">← Voltar</button>
    </header>

    <!-- Card único: Alertas Ativos -->
    <section class="cards cards-single">
      <div class="card border-left-red">
        <p class="label">Alertas Ativos</p>
        <p class="value">{{ alertas.length }}</p>
        <img src="@/assets/exclamation-mark.svg" alt="Alerta" class="icon-img" />
      </div>
    </section>

    <!-- Lista de Alertas -->
    <h2 class="list-title">Lista de Alertas</h2>

    <!-- Campo de Busca e Filtro -->
    <div class="search-filter">
      <div class="search-box">
        <input
          v-model="busca"
          type="text"
          placeholder="🔍 Buscar por nome do equipamento..."
          class="search-input"
        />
      </div>

      <select v-model="tipoSelecionado" class="filter-select">
        <option value="">Todos os Tipos</option>
        <option value="Microcontrolador">Microcontrolador</option>
        <option value="Equipamento">Equipamento</option>
        <option value="Componente">Componente</option>
        <option value="Outros">Outros</option>
      </select>
    </div>

    <!-- Alertas -->
    <div v-if="carregando">Carregando alertas...</div>
    <div v-else-if="erro">{{ erro }}</div>
    <div v-else>
      <div v-for="(alerta, index) in alertasFiltrados" :key="index" class="alert-card">
        <div class="alert-header">
          <h3>{{ alerta.name }}</h3>
          <div class="tags">
            <span class="badge red">Ativo</span>
            <span class="badge gray">{{ alerta.type || 'Indefinida' }}</span>
          </div>
        </div>

        <div class="alert-info">
          <p><strong>Tipo:</strong> {{ alerta.type }}</p>
          <p><strong>Local:</strong> {{ alerta.location }}</p>
          <p><strong>Data/Hora:</strong> {{ formatarData(alerta.updatedAt) }}</p>
          <p><strong>Responsável:</strong> {{ alerta.responsible }}</p>
        </div>

        <div class="alert-description">
          <p><strong>Descrição:</strong> Equipamento fora de alcance</p>
          <p><strong>Ação Recomendada:</strong> Verificar localização e status do dispositivo RFID</p>
        </div>

        <div class="alert-actions">
          <button class="resolve-btn" @click="changeStatus(alerta)">
            <img src="@/assets/check-mark.svg" alt="Resolver" class="btn-icon"/>
            Resolver
          </button>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, computed } from 'vue'
import { useRouter } from 'vue-router'
import { useUserStore } from '@/stores/user'
import api from '@/api/api'

// Tipagem para equipamentos
interface Equipment {
  id: number;
  name: string;
  type: string;
  location: string;
  rfid: string;
  responsible: string;
  isInRange: boolean;
  updatedAt: string;
}

interface EquipmentApiResponse {
  data: Equipment[];
  total: number;
  page: number;
  limit: number;
  totalPages: number;
}

const router = useRouter()
const userStore = useUserStore()

const equipamentos = ref<Equipment[]>([])
const alertas = ref<Equipment[]>([])
const carregando = ref(true)
const erro = ref<string | null>(null)

const busca = ref('')
const tipoSelecionado = ref('')

const voltar = () => {
  router.push({ path: '/' })
}

const formatarData = (data: string): string => {
  return new Date(data).toLocaleString('pt-BR')
}

const alertasFiltrados = computed(() =>
  alertas.value.filter((a) => {
    const nomeMatch = a.name.toLowerCase().includes(busca.value.toLowerCase())
    const tipoMatch = tipoSelecionado.value === '' || a.type === tipoSelecionado.value
    return nomeMatch && tipoMatch
  })
)

async function changeStatus(equip: Equipment) {
  try {
    console.log('Token sendo enviado:', userStore.token);
    await api.put(`equipament/change-status/${equip.rfid}`,{
      headers: {
        Authorization: `Bearer ${userStore.token}`
      }
    })
  } catch (erro) {
    console.log(`Erro ao mudar o status ${erro}`)
  }
  await carregarEquipamentos();
}

async function carregarEquipamentos() {
  carregando.value = true
  erro.value = null
  try {
    const response = await api.get<EquipmentApiResponse>('/equipament', {
      headers: {
        Authorization: `Bearer ${userStore.token}`
      }
    })

    equipamentos.value = response.data.data
    alertas.value = equipamentos.value.filter(eq => !eq.isInRange)
  } catch (err: any) {
    erro.value = err.response?.data?.message || 'Erro ao carregar alertas.'
  } finally {
    carregando.value = false
  }
}

onMounted(() => {
  carregarEquipamentos()
})
</script>

<style scoped>
.alert-view {
  padding: 1.5rem;
  background: #f9fafb;
  min-height: 100vh;
  font-family: 'Arial', sans-serif;
}

.top-bar {
  display: flex;
  justify-content: space-between;
  align-items: flex-start;
  margin-bottom: 2rem;
  position: relative;
}

.title-area {
  text-align: left;
}
.main-title {
  font-size: 1.4rem;
  font-weight: bold;
  margin: 0;
}
.subtitle {
  font-size: 0.85rem;
  color: #6b7280;
}

.btn-back {
  background: white;
  border: 1px solid #d1d5db;
  padding: 0.5rem 1rem;
  border-radius: 8px;
  cursor: pointer;
  font-size: 0.9rem;
}

.cards {
  display: flex;
  gap: 1rem;
  margin: 1rem 0;
  flex-wrap: wrap;
}
.cards-single {
  max-width: 400px;
  min-width: 280px;
}

.card {
  flex: 1;
  padding: 1rem;
  border-radius: 12px;
  background: white;
  color: #111827;
  min-width: 180px;
  position: relative;
  border-left: 6px solid transparent;
  box-shadow: 0 1px 3px rgba(0, 0, 0, 0.05);
}
.card .label {
  font-size: 0.9rem;
  margin-bottom: 0.2rem;
}
.card .value {
  font-size: 1.4rem;
  font-weight: bold;
}
.icon-img {
  width: 24px;
  height: 24px;
  position: absolute;
  top: 1rem;
  right: 1rem;
}

.btn-icon {
  width: 16px;
  height: 16px;
  margin-right: 6px;
  vertical-align: middle;
}

.border-left-red { border-left-color: #ef4444; }

.list-title {
  font-size: 1.3rem;
  font-weight: bold;
  margin-bottom: 1rem;
}

.search-filter {
  display: flex;
  flex-wrap: wrap;
  gap: 1rem;
  margin-bottom: 1.5rem;
  align-items: center;
}

.search-box {
  flex-grow: 1;
}

.search-input {
  width: 100%;
  padding: 0.6rem 0.75rem;
  border: 1px solid #d1d5db;
  border-radius: 6px;
  font-size: 0.85rem;
}

.filter-select {
  padding: 0.6rem;
  border: 1px solid #d1d5db;
  border-radius: 6px;
  font-size: 0.85rem;
}

.alert-card {
  background: white;
  border-radius: 8px;
  box-shadow: 0 1px 4px rgba(0,0,0,0.05);
  padding: 1rem;
  margin-bottom: 1rem;
}
.alert-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
}
.tags {
  display: flex;
  gap: 0.5rem;
}
.badge {
  font-size: 0.75rem;
  padding: 0.2rem 0.5rem;
  border-radius: 999px;
  font-weight: bold;
}
.badge.red { background: #fee2e2; color: #b91c1c; }
.badge.gray { background: #e5e7eb; color: #374151; }

.alert-info, .alert-description {
  margin-top: 0.5rem;
  font-size: 0.85rem;
}
.alert-info p, .alert-description p {
  margin: 0.3rem 0;
}

.alert-actions {
  margin-top: 1rem;
  display: flex;
  gap: 0.5rem;
}
.resolve-btn {
  border: none;
  border-radius: 6px;
  padding: 0.4rem 0.7rem;
  font-size: 0.8rem;
  cursor: pointer;
  background: #10b981;
  color: white;
}
</style>
