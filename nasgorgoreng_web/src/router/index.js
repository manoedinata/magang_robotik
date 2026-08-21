import { createRouter, createWebHistory } from 'vue-router'
import { useAuth } from '@/stores/auth'

import HomeView from '../views/HomeView.vue'

const router = createRouter({
  history: createWebHistory(import.meta.env.BASE_URL),
  routes: [
    {
      path: '/',
      name: 'home',
      component: HomeView,
    },
    {
      path: '/login',
      name: 'login',
      // route level code-splitting
      // this generates a separate chunk (About.[hash].js) for this route
      // which is lazy-loaded when the route is visited.
      component: () => import('../views/LoginView.vue'),
      // meta: { guestOnly: true },
    },
    {
      path: '/station',
      name: 'station',
      // route level code-splitting
      // this generates a separate chunk (About.[hash].js) for this route
      // which is lazy-loaded when the route is visited.
      component: () => import('../views/StationView.vue'),
      meta: { requiresAuth: true },
    },
  ],
})

router.beforeEach((to, from, next) => {
  const { auth } = useAuth() // Get the reactive auth state

  const requiresAuth = to.meta.requiresAuth
  const guestOnly = to.meta.guestOnly

  // If route requires auth and user is not logged in,
  // redirect to login page.
  if (requiresAuth && !auth.isAuthenticated) {
    next({ name: 'login' })
  }

  // If route is for guests only (like login page) and user
  // IS logged in, redirect them away from login.
  // else if (guestOnly && auth.isAuthenticated) {
  //   next({ name: 'station' })
  // }
  else {
    // Otherwise, proceed as normal.
    next()
  }
})

export default router
