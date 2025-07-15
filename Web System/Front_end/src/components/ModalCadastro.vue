<template>
  <div class="modal-overlay" @click.self="$emit('close')">
    <div class="modal-content">
      <h2>{{ isEdicao ? 'Editar Equipamento' : 'Cadastrar Novo Equipamento' }}</h2>
      <p class="subtitle">
        {{ isEdicao
          ? 'Atualize as informações do equipamento'
          : 'Adicione um novo equipamento ao sistema de monitoramento' }}
      </p>

      <form @submit.prevent="handleSubmit">
        <label>Nome do Equipamento *</label>
        <input
          type="text"
          placeholder="Ex: Placa Beagle Bone Black nº11"
          v-model="form.nome"
          required
        />

        <label>Tipo *</label>
        <select v-model="form.tipo" required>
          <option disabled value="">Selecione o tipo</option>
          <option>Microcontrolador</option>
          <option>FPGA</option>
          <option>Protoboard</option>
          <option>Ociloscópio</option>
          <option>Fonte</option>
          <option>Multímetro</option>
          <option>embarcado</option>
          <option>Outro</option>
        </select>

        <label>Localização *</label>
        <select v-model="form.local" required>
          <option disabled value="">Selecione a localização</option>
          <option>Almoxarifado</option>
          <option>Laboratório de Robótica</option>
          <option>Laboratório de Eletrônica</option>
          <option>Laboratório 2</option>
        </select>

        <label>Código da Tag RFID *</label>
        <input
          type="text"
          placeholder="Ex: RFID001234"
          v-model="form.rfid"
          required
        />

        <label>Responsável</label>
        <input
          type="text"
          placeholder="Ex: Prof. Silva"
          v-model="form.responsavel"
        />

        <div class="buttons">
          <button type="button" class="cancel" @click="$emit('close')">Cancelar</button>
          <button type="submit" class="submit">
            {{ isEdicao ? 'Salvar Alterações' : 'Cadastrar Equipamento' }}
          </button>
        </div>
      </form>
    </div>
  </div>
</template>

<script setup lang="ts">
import { reactive, computed, watch } from 'vue'

const props = defineProps<{
  equipamento?: {
    id?: number
    name: string
    type: string
    location: string
    rfid: string
    responsible: string
  } | null
}>()

const emit = defineEmits<{
  (e: 'close'): void
  (e: 'save', data: any): void
}>()

const form = reactive({
  nome: '',
  tipo: '',
  local: '',
  rfid: '',
  responsavel: '',
})

const isEdicao = computed(() => !!props.equipamento)

watch(
  () => props.equipamento,
  (equip) => {
    if (equip) {
      form.nome = equip.name
      form.tipo = equip.type
      form.local = equip.location
      form.rfid = equip.rfid
      form.responsavel = equip.responsible
    } else {
      // Limpando se for criação
      form.nome = ''
      form.tipo = ''
      form.local = ''
      form.rfid = ''
      form.responsavel = ''
    }
  },
  { immediate: true }
)

function handleSubmit() {
  const equipamentoFinal = {
    id: props.equipamento?.id, // se for edição
    name: form.nome,
    type: form.tipo,
    location: form.local,
    rfid: form.rfid,
    responsible: form.responsavel,
  }

  emit('save', equipamentoFinal)
  emit('close')
}
</script>

<style scoped>
.modal-overlay {
  position: fixed;
  inset: 0;
  background: rgba(0, 0, 0, 0.4);
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 50;
}

.modal-content {
  background: white;
  padding: 2rem;
  border-radius: 1rem;
  width: 100%;
  max-width: 480px;
  box-shadow: 0 8px 20px rgba(0, 0, 0, 0.2);
}

.subtitle {
  font-size: 0.9rem;
  color: #555;
  margin-bottom: 1rem;
}

input,
select {
  width: 100%;
  padding: 0.6rem;
  margin: 0.5rem 0 1rem;
  border-radius: 0.5rem;
  border: 1px solid #ccc;
}

.buttons {
  display: flex;
  justify-content: flex-end;
  gap: 0.5rem;
}

button.cancel {
  background: #f3f3f3;
  color: #333;
}

button.submit {
  background: #1d4ed8;
  color: white;
}

button {
  padding: 0.6rem 1rem;
  border: none;
  border-radius: 0.5rem;
  cursor: pointer;
}
</style>
