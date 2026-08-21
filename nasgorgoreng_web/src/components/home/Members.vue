<script setup>
defineProps({
  name: {
    type: String,
    required: true,
  },
  major: {
    type: String,
    required: true,
  },
  role: {
    type: String,
    required: true,
  },
  instagram: {
    type: String,
    required: true,
  },
  image: {
    type: String,
    required: true,
  },
})
</script>

<template>
  <div class="profile-card-2">
    <img :src="image" class="img img-fluid" />
    <div class="profile-name">{{ name }}</div>
    <div class="profile-username">{{ major }} / {{ role }}</div>
    <div class="profile-icons">
      <a :href="`https://instagram.com/${instagram}`"><i class="bi bi-instagram"></i></a>
    </div>
  </div>
</template>
