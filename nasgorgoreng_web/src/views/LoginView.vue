<script setup>
import { ref } from 'vue'
import { useAuth } from '@/stores/auth.js'

const { login } = useAuth()

const username = ref('')
const password = ref('')
const errorMessage = ref(null)

function handleLogin() {
  errorMessage.value = null // Clear previous errors
  const success = login(username.value, password.value)

  if (!success) {
    errorMessage.value = 'Invalid username or password.'
  }
}
</script>

<template>
  <div class="login-container d-flex align-items-center justify-content-center vh-100">
    <div class="card" style="width: 22rem">
      <div class="card-body">
        <h3 class="card-title text-center mb-4">
          <router-link to="/">
            <img src="/logo-dark.png" alt="NasgorGoreng Logo" class="img-fluid" />
          </router-link>
        </h3>

        <form @submit.prevent="handleLogin">
          <div class="mb-3">
            <label for="username" class="form-label">Username</label>
            <input type="text" class="form-control" id="username" v-model="username" required />
          </div>
          <div class="mb-3">
            <label for="password" class="form-label">Password</label>
            <input type="password" class="form-control" id="password" v-model="password" required />
          </div>

          <div v-if="errorMessage" class="alert alert-danger p-2" role="alert">
            {{ errorMessage }}
          </div>

          <button type="submit" class="btn btn-info w-100">Login</button>
        </form>
      </div>
    </div>
  </div>
</template>
