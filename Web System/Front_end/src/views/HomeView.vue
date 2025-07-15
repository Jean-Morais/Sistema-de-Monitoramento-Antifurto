<template>
  <div class="dashboard-container">
    <header class="header">
      <div>
        <h1 class="title">SecureTrack Dashboard</h1>
        <p class="subtitle">Sistema de Monitoramento Anti-Furto</p>
      </div>
      <div class="header-actions">
        <span class="status-indicator">🟢 Sistema Online</span>
        <button class="btn logout" @click="handleLogout">Sair</button>
      </div>
    </header>

    <section class="cards">
      <router-link to="/gerenciamento" class="card-link">
        <div class="card border-blue">
          <div class="card-content">
            <div class="card-text">
              <p class="card-title">Equipamentos Monitorados</p>
              <p class="card-value">{{ equipments.length }}</p>
            </div>
            <div class="card-icon">📦</div>
          </div>
        </div>
      </router-link>

      <router-link to="/alertas" class="card border-red card-link">
        <div class="card-content">
          <div class="card-text">
            <p class="card-title">Gerenciador de Alertas</p>
            <p class="card-value">{{ outOfRangeEquipmentsCount }}</p>
          </div>
          <img src="@/assets/exclamation-mark.svg" alt="Alerta" class="card-icon-img" />
        </div>
      </router-link>

      <div class="card border-green">
        <div class="card-content">
          <div class="card-text">
            <p class="card-title">Equipamentos Online</p>
            <p class="card-value">{{ inRangeEquipmentsCount }}</p>
          </div>
          <img src="@/assets/check-mark.svg" alt="Equipamento Online" class="card-icon-img" />
        </div>
      </div>
    </section>

    <!--
    <section class="alerts">
      <h2 class="section-title">Alertas Recentes</h2>
      <table>
        <thead>
          <tr>
            <th>Equipamento</th>
            <th>Tipo</th>
            <th>Localização</th>
            <th>Data/Hora</th>
            <th>Status</th>
            <th>Prioridade</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="(alerta, i) in alertas" :key="i">
            <td>{{ alerta.equipamento }}</td>
            <td>{{ alerta.tipo }}</td>
            <td>{{ alerta.localizacao }}</td>
            <td>{{ alerta.data }}</td>
            <td>{{ alerta.status }}</td>
            <td>{{ alerta.prioridade }}</td>
          </tr>
        </tbody>
      </table>
    </section>
    -->
<section class="equipments">
      <h2 class="section-title">Todos os Equipamentos</h2>
      <div v-if="loadingEquipments">Carregando equipamentos...</div>
      <div v-else-if="equipmentError">Erro ao carregar equipamentos: {{ equipmentError }}</div>
      <div v-else-if="equipments.length === 0">Nenhum equipamento encontrado.</div>
      <table v-else>
        <thead>
          <tr>
            <th>ID</th>
            <th>Nome</th>
            <th>Tipo</th>
            <th>Localização</th>
            <th>RFID</th>
            <th>Responsável</th>
            <th>Status</th>
            <th>Última Atualização</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="equipment in equipments" :key="equipment.id">
            <td>{{ equipment.id }}</td>
            <td>{{ equipment.name }}</td>
            <td>{{ equipment.type }}</td>
            <td>{{ equipment.location }}</td>
            <td>{{ equipment.rfid }}</td>
            <td>{{ equipment.responsible }}</td>
            <td>{{ equipment.isInRange ? 'Em alcance' : 'Fora de alcance' }}</td>
            <td>{{ new Date(equipment.updatedAt).toLocaleString() }}</td>
          </tr>
        </tbody>
      </table>
    </section>
  </div>
</template>

<script setup lang="ts"> // Adicionado lang="ts" para tipagem
import { ref, onMounted, computed } from 'vue'
import { useRouter } from 'vue-router'
import { useUserStore } from '@/stores/user'
import api from '@/api/api' // Importe sua instância do axios ou fetch configurado

// Tipagem para um equipamento
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

// Interface para a resposta completa da API de equipamentos
interface EquipmentApiResponse {
  data: Equipment[];
  total: number;
  page: number;
  limit: number;
  totalPages: number;
}


const userStore = useUserStore()
const router = useRouter()

/* 
const alertas = [
  {
    equipamento: 'Placa Beagle Bone Black nº10',
    tipo: 'Movimento Indevido',
    localizacao: 'Escada do Corredor',
    data: '2024-01-15 14:32:15',
    status: 'ativo',
    prioridade: 'alta',
  },
  {
    equipamento: 'Osciloscópio Tektronix TDS2024B',
    tipo: 'Saída Não Autorizada',
    localizacao: 'Laboratório de Eletrônica',
    data: '2024-01-15 13:15:42',
    status: 'resolvido',
    prioridade: 'média',
  },
  {
    equipamento: 'Placa Zybo Z7-20',
    tipo: 'Dispositivo Inativo',
    localizacao: 'Laboratório de Robótica',
    data: '2024-01-15 12:45:20',
    status: 'ativo',
    prioridade: 'baixa',
  },
]
*/

// Novas variáveis para equipamentos
const equipments = ref<Equipment[]>([])
const loadingEquipments = ref(true)
const equipmentError = ref<string | null>(null)

// Computed Property para contar equipamentos fora de alcance
const outOfRangeEquipmentsCount = computed(() => {
  return equipments.value.filter(eq => !eq.isInRange).length;
});

// Opcional: Computed Property para contar equipamentos em alcance (online)
const inRangeEquipmentsCount = computed(() => {
  return equipments.value.filter(eq => eq.isInRange).length;
});

const handleLogout = () => {
  userStore.clearUserData()
  router.push('/login')
}

async function fetchEquipments() {
  loadingEquipments.value = true
  equipmentError.value = null
  try {
    if (!userStore.token) {
      throw new Error('Token de autenticação não encontrado.')
    }

    const response = await api.get<EquipmentApiResponse>('/equipament', { // Rota correta com /api/
      headers: {
        Authorization: `Bearer ${userStore.token}`
      }
    })

    equipments.value = response.data.data
  } catch (error: any) {
    console.error('Erro ao buscar equipamentos:', error)
    if (error.response && error.response.status === 401) {
      equipmentError.value = 'Sessão expirada. Por favor, faça login novamente.'
      handleLogout()
    } else {
      equipmentError.value = error.message || 'Ocorreu um erro desconhecido.'
    }
  } finally {
    loadingEquipments.value = false
  }
}

// Chama a função ao montar o componente
onMounted(() => {
  fetchEquipments()
})
</script>

<style scoped>
* {
  font-family: "Inter", sans-serif;
}

.dashboard-container {
  padding: 1.5rem;
  background: #f9fafb;
  min-height: 100vh;
}

/* ======= Header ======= */
.header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 1.5rem;
  flex-wrap: wrap;
  gap: 1rem;
}

.title {
  font-size: 1.4rem;
  font-weight: bold;
  color: #1e40af;
}

.subtitle {
  font-size: 0.85rem;
  color: #555;
}

.header-actions {
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

.status-indicator {
  font-size: 0.85rem;
  color: #059669;
}

.btn {
  padding: 0.4rem 0.7rem;
  border-radius: 0.4rem;
  font-size: 0.8rem;
  cursor: pointer;
  background-color: transparent;
  border: 1px solid transparent;
}

.logout {
  border-color: #ef4444;
  color: #ef4444;
}

/* ======= Cards com borda lateral ======= */
.cards {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(240px, 1fr));
  gap: 1.2rem;
  margin-bottom: 2rem;
}

.card {
  padding: 1.2rem;
  border-radius: 0.75rem;
  background: white;
  color: #1f2937;
  box-shadow: 0 1px 4px rgba(0, 0, 0, 0.06);
  display: flex;
  align-items: center;
  transition: transform 0.2s ease;
  border-left: 5px solid transparent;
}

.card:hover {
  transform: translateY(-3px);
}

.card-content {
  display: flex;
  justify-content: space-between;
  align-items: center;
  width: 100%;
}

.card-title {
  font-size: 0.85rem;
  color: #4b5563;
}

.card-value {
  font-size: 1.8rem;
  font-weight: bold;
  margin-top: 0.3rem;
  color: #1f2937;
}

.card-icon {
  font-size: 1.8rem;
}

.card-icon-img {
  width: 32px;
  height: 32px;
}

.card-link {
  text-decoration: none;
  color: inherit;
}

/* Cores das bordas */
.border-blue {
  border-left-color: #3b82f6;
}

.border-red {
  border-left-color: #ef4444;
}

.border-green {
  border-left-color: #10b981;
}

/* ======= Tabela de Alertas ======= */
.alerts {
  background: white;
  border-radius: 1rem;
  padding: 1rem;
  box-shadow: 0 2px 6px rgba(0, 0, 0, 0.05);
}

.section-title {
  font-size: 1.1rem;
  font-weight: 600;
  margin-bottom: 1rem;
}

table {
  width: 100%;
  border-collapse: collapse;
  font-size: 0.85rem;
}

thead {
  background: #f3f4f6;
}

th,
td {
  padding: 0.75rem;
  text-align: left;
}

tbody tr {
  border-top: 1px solid #e5e7eb;
}
</style>
