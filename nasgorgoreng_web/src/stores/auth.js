import { reactive, watch } from 'vue'
import { useRouter } from 'vue-router'

// Define the credentials
const APP_USERNAME = import.meta.env.VITE_APP_USERNAME || 'admin'
const APP_PASSWORD = import.meta.env.VITE_APP_PASSWORD || 'password123'

// Create a reactive object to hold the auth state
// Initialize 'isAuthenticated' from localStorage
const auth = reactive({
  isAuthenticated: localStorage.getItem('isAuthenticated') === 'true',
})

// Watch for changes to 'isAuthenticated' and update localStorage
watch(
  () => auth.isAuthenticated,
  (newVal) => {
    localStorage.setItem('isAuthenticated', newVal.toString())
  },
)

/**
 * A composable function to provide auth state and methods.
 */
export function useAuth() {
  const router = useRouter()

  /**
   * Attempts to log in with the given credentials.
   * @param {string} username
   * @param {string} password
   * @returns {boolean} True if login was successful, false otherwise.
   */
  function login(username, password) {
    if (username === APP_USERNAME && password === APP_PASSWORD) {
      auth.isAuthenticated = true
      // Redirect to the protected page after successful login
      router.push({ name: 'station' }) // Assuming your station route is named 'station'
      return true
    }
    return false
  }

  /**
   * Logs the user out.
   */
  function logout() {
    auth.isAuthenticated = false
    // Redirect to the login page after logout
    router.push({ name: 'login' })
  }

  return {
    auth,
    login,
    logout,
  }
}
