<script setup lang="ts">
import { computed } from 'vue'
import Controller from './Controller.vue'

const props = defineProps<{ gamepad: Gamepad }>()
</script>

<template>
  <div
    bg="dark:dark-500 light-100"
    shadow="~ md"
    border="rounded"
    max-w="screen-lg"
    mx="auto"
    overflow="hidden"
    grid="~"
    class="grid-cols-[2fr,1fr]"
  >
    <div
      flex="~ row shrink-none"
      place="items-center content-center"
      p="8"
      bg="dark:dark-900 light-400"
    >
      <template v-if="gamepad.mapping === 'standard'">
        <Controller :gamepad="gamepad" text="dark-100 opacity-70 dark:(light-900 opacity-70)" />
      </template>
      <template v-else>
        <span font-medium text="dark:(light-900 opacity-70)"> Unknown Controller Type </span>
      </template>
    </div>
  </div>
</template>
