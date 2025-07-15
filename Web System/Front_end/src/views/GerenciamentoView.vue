<template>
  <div class="container">
    <!-- Cabeçalho -->
    <header class="page-header">
      <div class="header-text">
        <h1>Gerenciar Equipamentos</h1>
        <p class="subtitle">Cadastro e monitoramento de itens patrimoniais</p>
      </div>
      <div class="header-buttons">
        <button class="new-button" @click="abrirCriacao">+ Novo Equipamento</button>
        <button class="back-button" @click="handleVoltar">Voltar</button>
      </div>
    </header>

    <!-- Filtros -->
    <section class="filtros">
      <h2>Filtros e Busca</h2>
      <div class="filtro-inputs">
        <input
          type="text"
          class="input-busca"
          placeholder="Buscar por nome, localização ou tag RFID..."
          v-model="search"
        />
        <select class="select-tipo" v-model="tipoFiltro">
          <option value="">Todos os tipos</option>
          <option>Microcontrolador</option>
          <option>FPGA</option>
          <option>Protoboard</option>
        </select>
      </div>
    </section>

    <!-- Lista de Equipamentos -->
    <section class="equipamentos">
      <h2>Lista de Equipamentos</h2>
      <p class="equip-count">{{ equipamentosFiltrados.length }} equipamentos encontrados</p>
      <div v-if="carregando">Carregando equipamentos...</div>
      <div v-else-if="erro">Erro ao carregar: {{ erro }}</div>
      <div v-else-if="equipamentosFiltrados.length === 0">Nenhum equipamento encontrado.</div>
      <table v-else>
        <thead>
          <tr>
            <th>Equipamento</th>
            <th>Tipo</th>
            <th>Localização</th>
            <th>Tag RFID</th>
            <th>Status</th>
            <th>Última Atividade</th>
            <th>Responsável</th>
            <th>Ações</th>
          </tr>
        </thead>
        <tbody>
          <tr v-for="equip in equipamentosFiltrados" :key="equip.id">
            <td>{{ equip.name }}</td>
            <td><span class="badge">{{ equip.type }}</span></td>
            <td>{{ equip.location }}</td>
            <td>{{ equip.rfid }}</td>
            <td>
              <span class="status ativo" v-if="equip.isInRange">Ativo</span>
              <span class="status inativo" v-else>Inativo</span>
            </td>
            <td>{{ new Date(equip.updatedAt).toLocaleString() }}</td>
            <td>{{ equip.responsible }}</td>
            <td class="action-buttons">
              <button class="icon-button" @click="abrirEdicao(equip)">
                <img src="@/assets/edit.svg" alt="Editar" />
              </button>
              <button class="icon-button" @click="deletarEquipamento(equip)">
                <img src="@/assets/delete.svg" alt="Excluir" />
              </button>
            </td>
          </tr>
        </tbody>
      </table>
    </section>

    <ModalCadastro
      v-if="showModal"
      :equipamento="equipamentoEditando"
      @close="fecharModal"
      @save="salvarEquipamento"
    />
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted } from 'vue'
import ModalCadastro from '../components/ModalCadastro.vue'
import api from '@/api/api'
import { useUserStore } from '@/stores/user'
import { useRouter } from 'vue-router'

const router = useRouter()
const userStore = useUserStore()

const showModal = ref(false)
const equipamentoEditando = ref<Equipment | null>(null)

const equipamentos = ref<Equipment[]>([])
const carregando = ref(true)
const erro = ref<string | null>(null)
const search = ref('')
const tipoFiltro = ref('')

interface Equipment {
  id: number
  name: string
  type: string
  location: string
  rfid: string
  responsible: string
  isInRange: boolean
  updatedAt: string
}

const equipamentosFiltrados = computed(() =>
  equipamentos.value.filter((equip) => {
    const busca = search.value.toLowerCase()
    const nomeMatch = equip.name.toLowerCase().includes(busca)
    const localMatch = equip.location.toLowerCase().includes(busca)
    const rfidMatch = equip.rfid.toLowerCase().includes(busca)
    const tipoMatch = tipoFiltro.value ? equip.type === tipoFiltro.value : true
    return (nomeMatch || localMatch || rfidMatch) && tipoMatch
  })
)

function handleVoltar() {
  router.push('/')
}

function abrirCriacao() {
  equipamentoEditando.value = null
  showModal.value = true
}

function abrirEdicao(equip: Equipment) {
  equipamentoEditando.value = equip
  showModal.value = true
}

function fecharModal() {
  showModal.value = false
  equipamentoEditando.value = null
}

async function deletarEquipamento(equip: any) {
  try {
    await api.delete(`/equipament/${equip.rfid}`,{
        headers: {
          Authorization: `Bearer ${userStore.token}`
        }
      });
  } catch (erro) {
    console.log(`Erro ao deletar ${erro}`)
  }
  await carregarEquipamentos();
}

async function salvarEquipamento(equip: any) {
  try {
    if (equip.id) {
      // PUT - Atualizar
      await api.put(`/equipament/${equip.rfid}`, equip, {
        headers: {
          Authorization: `Bearer ${userStore.token}`
        }
      })
    } else {
      // POST - Criar novo
      await api.post('/equipament', equip, {
        headers: {
          Authorization: `Bearer ${userStore.token}`
        }
      })
    }
    await carregarEquipamentos()
  } catch (error: any) {
    console.error('Erro ao salvar equipamento:', error)
  }
}

async function carregarEquipamentos() {
  carregando.value = true
  erro.value = null
  try {
    const response = await api.get('/equipament', {
      headers: {
        Authorization: `Bearer ${userStore.token}`
      }
    })
    equipamentos.value = response.data.data
  } catch (err: any) {
    erro.value = err.message || 'Erro inesperado ao buscar equipamentos.'
  } finally {
    carregando.value = false
  }
}

onMounted(() => {
  carregarEquipamentos()
})
</script>

<style scoped>
.container {
  padding: 2rem;
  font-family: Inter, sans-serif;
  background-color: #fff;
}

/* Cabeçalho */
.page-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 2rem;
}

.header-text h1 {
  margin: 0;
  font-size: 1.5rem;
}

.subtitle {
  font-size: 0.875rem;
  color: #666;
  margin: 0;
}

.header-buttons {
  display: flex;
  gap: 0.5rem;
}

.back-button,
.new-button {
  background-color: #2563eb;
  color: white;
  border: none;
  border-radius: 0.5rem;
  padding: 0.5rem 1rem;
  cursor: pointer;
}

/* Filtros */
.filtros {
  background: #f9f9f9;
  padding: 1rem;
  border-radius: 0.75rem;
  margin-bottom: 2rem;
}

.filtro-inputs {
  display: flex;
  align-items: center;
  gap: 1rem;
  margin-top: 1rem;
}

.input-busca {
  flex: 1;
  padding: 0.5rem;
  border-radius: 0.5rem;
  border: 1px solid #ccc;
}

.select-tipo {
  width: 200px;
  padding: 0.5rem;
  border-radius: 0.5rem;
  border: 1px solid #ccc;
  background-color: #f3f3f3;
}

/* Tabela */
.equipamentos table {
  width: 100%;
  border-collapse: collapse;
  margin-top: 1rem;
}

.equipamentos th,
.equipamentos td {
  padding: 0.75rem;
  text-align: left;
  border-bottom: 1px solid #eee;
}

.badge {
  background: #e0e7ff;
  color: #1d4ed8;
  padding: 0.25rem 0.5rem;
  border-radius: 0.5rem;
  font-size: 0.75rem;
}

.status.ativo {
  background: #bbf7d0;
  color: #15803d;
  padding: 0.25rem 0.5rem;
  border-radius: 0.5rem;
  font-size: 0.75rem;
}

.status.inativo {
  background: #fecaca;
  color: #b91c1c;
  padding: 0.25rem 0.5rem;
  border-radius: 0.5rem;
  font-size: 0.75rem;
}
.action-buttons {
  display: flex;
  gap: 0.5rem;
}

.icon-button {
  background: transparent;
  border: none;
  cursor: pointer;
  padding: 0.3rem;
  display: flex;
  align-items: center;
  justify-content: center;
}

.icon-button img {
  width: 20px;
  height: 20px;
}

</style>

