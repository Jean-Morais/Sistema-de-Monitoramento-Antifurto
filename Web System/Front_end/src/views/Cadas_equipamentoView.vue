<template>
  <div class="modal-overlay">
    <div class="modal-content">
      <h2 class="modal-title">Cadastrar Novo Equipamento</h2>
      <p class="modal-subtitle">Adicione um novo equipamento ao sistema de monitoramento</p>

      <form class="form" @submit.prevent="handleSubmit">
        <div class="form-group">
          <label>Nome do Equipamento *</label>
          <input type="text" placeholder="Ex: Placa Beagle Bone Black nº11" v-model="nome" required />
        </div>

        <div class="form-group">
          <label>Tipo *</label>
          <select v-model="tipo" required>
            <option value="" disabled selected>Selecione o tipo</option>
            <option>Microcontrolador</option>
            <option>FPGA</option>
            <option>Osciloscópio</option>
            <option>Fonte de Alimentação</option>
            <option>Multímetro</option>
            <option>Gerador de função</option>
            <option>Outros</option>
          </select>
        </div>

        <div class="form-group">
          <label>Localização *</label>
          <select v-model="localizacao" required>
            <option value="" disabled selected>Selecione a localização</option>
            <option>Amoxarifado</option>
            <option>Laboratório de Arquitetura</option>
            <option>laboratóriio de Eletrônica</option>
            <option>Laboratório de Robótica</option>
            <option>Outros</option>

          </select>
        </div>

        <div class="form-group">
          <label>Código da Tag RFID *</label>
          <input type="text" placeholder="Ex: RFID001234" v-model="tag" required />
        </div>

        <div class="form-group">
          <label>Responsável</label>
          <input type="text" placeholder="Ex: Prof. Silva" v-model="responsavel" />
        </div>

        <div class="button-group">
          <button type="button" class="cancel" @click="handleCancel">Cancelar</button>
          <button path="/dashboard"
          type="submit" class="confirm">Cadastrar Equipamento</button>
        </div>
      </form>
    </div>
  </div>
</template>

<script lang="ts" setup>
import { ref } from 'vue'

const nome = ref('')
const tipo = ref('')
const localizacao = ref('') 
const tag = ref('')
const responsavel = ref('')

function handleSubmit() {
  console.log('Equipamento cadastrado:', {
    nome: nome.value,
    tipo: tipo.value,
    localizacao: localizacao.value,
    tag: tag.value,
    responsavel: responsavel.value,
  })
  // Aqui você pode emitir evento para o pai ou chamar uma API
}

function handleCancel() {
  // Fechar o modal
  console.log('Cadastro cancelado')
}
</script>

<style scoped>
.modal-overlay {
  position: fixed;
  top: 0;
  left: 0;
  right: 0;
  bottom: 0;
  background: rgba(0, 0, 0, 0.4);
  display: flex;
  align-items: center;
  justify-content: center;
  z-index: 50;
}

.modal-content {
  background: #fff;
  padding: 2rem;
  border-radius: 0.75rem;
  width: 100%;
  max-width: 500px;
  box-shadow: 0 10px 25px rgba(0, 0, 0, 0.1);
}

.modal-title {
  font-size: 1.25rem;
  font-weight: 600;
  margin-bottom: 0.25rem;
}

.modal-subtitle {
  font-size: 0.9rem;
  color: #666;
  margin-bottom: 1.5rem;
}

.form-group {
  display: flex;
  flex-direction: column;
  margin-bottom: 1rem;
}
body {
  font-family: "Inter", sans-serif;
}

label {
  font-size: 0.85rem;
  margin-bottom: 0.25rem;
  font-weight: 500;
}

input,
select {
  padding: 0.6rem;
  border: 1px solid #ccc;
  border-radius: 0.5rem;
  font-size: 0.95rem;
}

input:focus,
select:focus {
  outline: none;
  border-color: #2563eb;
  box-shadow: 0 0 0 2px rgba(37, 99, 235, 0.2);
}

.button-group {
  display: flex;
  justify-content: flex-end;
  gap: 0.75rem;
  margin-top: 1.5rem;
}

button {
  padding: 0.6rem 1rem;
  border-radius: 0.5rem;
  font-size: 0.95rem;
  cursor: pointer;
  border: none;
  transition: background 0.2s;
}

.cancel {
  background: #f3f4f6;
  color: #111;
}

.confirm {
  background: #111827;
  color: white;
}

.confirm:hover {
  background: #1f2937;
}

.cancel:hover {
  background: #e5e7eb;
}
</style>
