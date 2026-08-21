<script setup>
import { ref, reactive, onMounted, watch, computed } from 'vue'
import 'roslib/build/roslib'
import Toastify from 'toastify-js'
import 'toastify-js/src/toastify.css'

import FPSCounter from '../utils/FPSCounter.js'

import { useAuth } from '@/stores/auth.js'
const { logout } = useAuth()

import { useGamepad } from '@vueuse/core'
const { gamepads, onConnected, onDisconnected } = useGamepad()
import Gamepad from '@/components/station/gamepad/Gamepad.vue'

const ros = ref(null)
const connected = ref(false)
const logs = ref([])
const serverURL = ref('ws://localhost:9090')
const started = ref(false)
const manualControl = ref(true)

// Canvas
const rawCanvasRef = ref(null)
const processedCanvasRef = ref(null)
const imageMetadata = ref({
  raw: { fps: 0, resolution: '-' },
  processed: { fps: 0, resolution: '-' },
})

// Topics
const imageRawTopic = ref('/camera/image_raw/compressed')
const imageProcessedTopic = ref('/camera/image_processed/compressed')
const telemetryTopic = ref('/vehicle/telemetry')
const sensorsTopic = ref('/sensors')
const manualControlTopic = ref('/manual_control')
const thresholdConfigTopic = ref('/threshold_config')
const decisionTopic = ref('/decision')

const imageRawListener = ref(null)
const imageProcessedListener = ref(null)
const telemetryListener = ref(null)
const sensorsListener = ref(null)
const manualControlListener = ref(null)
const thresholdConfigListener = ref(null)
const decisionListener = ref(null)

// FPS Counters
const rawFPSCounter = new FPSCounter()
const processedFPSCounter = new FPSCounter()

// Telemetry
const telemetry = reactive({
  steering_angle: 0,
  laneStatus: 'Unknown',
  speed: 0,
  obstacleDetected: 'None',
  obstacleDistance: 0,
})

// Threshold Settings
const thresholdSettings = reactive({
  canny_low_threshold: 152.0,
  canny_high_threshold: 255.0,
  bev_top_y_left_percent: 42,
  bev_top_y_right_percent: 42,
  bev_top_x_left_percent: 25,
  bev_top_x_right_percent: 90,
  bev_bottom_x_left_percent: 0,
  bev_bottom_x_right_percent: 200,
  cntr_left_lane_limit_center: 175,
})

function log(message, type = 'info') {
  const timestamp = new Date().toLocaleTimeString()
  logs.value.push({ timestamp, message, type })

  // Also log to console
  console.log(`[${timestamp}] ${message}`)
}

/**
 * Show notification toast
 */
function showNotification(message, type = 'success') {
  let notification = Toastify({
    text: message,
    duration: 3000,
    newWindow: true,
    close: true,
    gravity: 'top',
    position: 'right',
    stopOnFocus: false,
    style: {
      background:
        type === 'success'
          ? 'linear-gradient(to right, #00b09b, #96c93d)'
          : type === 'error'
            ? 'linear-gradient(to right, #e52d27, #b31217)'
            : 'linear-gradient(to right, #00b09b, #96c93d)',
    },
    onClick: function () {},
  })
  notification.showToast()
}

function displayImage(imageData, type, width = 0, height = 0) {
  const canvasRef = type === 'raw' ? rawCanvasRef : processedCanvasRef
  const canvas = canvasRef.value

  if (!canvas) {
    console.error(`Canvas element for type '${type}' is not yet available.`)
    return
  }

  // Get the 2D context
  const ctx = canvas.getContext('2d')

  if (width && height) {
    imageMetadata.value[type].resolution = `${width}x${height}`
  }

  const img = new Image()
  img.onload = () => {
    // Set canvas size to match image
    canvas.width = img.width
    canvas.height = img.height

    // Draw image
    ctx.drawImage(img, 0, 0)
  }

  // Handle both base64 with and without prefix
  if (imageData.startsWith('data:image')) {
    img.src = imageData
  } else {
    img.src = `data:image/jpeg;base64,${imageData}`
  }
}

function updateTelemetry(data) {
  Object.assign(telemetry, data)
}

function connectROS() {
  if (!window.ROSLIB) {
    log('Waiting for roslib to be loaded...', 'info')
    return
  }

  log(`Connecting to ROS bridge: ${serverURL.value}`, 'info')

  try {
    ros.value = new window.ROSLIB.Ros({
      url: serverURL.value,
    })

    ros.value.on('connection', () => {
      connected.value = true
      log('ROS bridge connected.', 'success')
      subscribeToROSTopics()
    })

    ros.value.on('error', (error) => {
      log(`ROS error: ${error}`, 'error')
    })

    ros.value.on('close', () => {
      connected.value = false
      log('ROS bridge disconnected', 'warning')
    })
  } catch (error) {
    log(`Failed to connect to ROS: ${error.message}`, 'error')
  }
}

function disconnectROS() {
  if (!ros.value) return

  ros.value.close()
  ros.value = null

  connected.value = false
  log('Disconnected from server', 'info')
  showNotification('ROS2 connection disconnected.', 'error')
}

function subscribeToROSTopics() {
  // Subscribe to raw image topic
  imageRawListener.value = new window.ROSLIB.Topic({
    ros: ros.value,
    name: imageRawTopic.value,
    messageType: 'sensor_msgs/CompressedImage',
  })

  imageRawListener.value.subscribe((message) => {
    displayImage(message.data, 'raw')
    rawFPSCounter.tick()
    imageMetadata.value.raw.fps = rawFPSCounter.fps
  })

  // Subscribe to processed image topic
  imageProcessedListener.value = new window.ROSLIB.Topic({
    ros: ros.value,
    name: imageProcessedTopic.value,
    messageType: 'sensor_msgs/CompressedImage',
  })

  imageProcessedListener.value.subscribe((message) => {
    displayImage(message.data, 'processed')
    processedFPSCounter.tick()
    imageMetadata.value.processed.fps = processedFPSCounter.fps
  })

  // Subscribe to telemetry topic
  telemetryListener.value = new window.ROSLIB.Topic({
    ros: ros.value,
    name: telemetryTopic.value,
    messageType: 'std_msgs/String',
  })

  telemetryListener.value.subscribe((message) => {
    const data = JSON.parse(message.data)
    telemetry.laneStatus = data.road_status
    telemetry.steering_angle = data.turn_angle
  })

  // Subscribe to sensors topic
  sensorsListener.value = new window.ROSLIB.Topic({
    ros: ros.value,
    name: sensorsTopic.value,
    messageType: 'std_msgs/String',
  })
  sensorsListener.value.subscribe((message) => {
    const data = JSON.parse(message.data)

    telemetry.obstacleDetected = data.obstacle_distance <= 30.0 ? 'Yes' : 'No'
    telemetry.obstacleDistance = data.obstacle_distance
    console.log(telemetry.obstacleDistance)
  })

  // Publish to manual control topic
  manualControlListener.value = new window.ROSLIB.Topic({
    ros: ros.value,
    name: manualControlTopic.value,
    messageType: 'std_msgs/Bool',
  })

  // Publish to threshold config topic
  thresholdConfigListener.value = new window.ROSLIB.Topic({
    ros: ros.value,
    name: thresholdConfigTopic.value,
    messageType: 'std_msgs/String',
  })

  // Publish to decision topic
  decisionListener.value = new window.ROSLIB.Topic({
    ros: ros.value,
    name: decisionTopic.value,
    messageType: 'std_msgs/String',
  })

  showNotification('ROS2 connection established.')
}

// Steering gauge update
watch(
  () => telemetry.steering_angle,
  (newAngle) => {
    // Update the gauge SVG
    const gaugeValueEl = document.getElementById('gaugeValue')
    const gaugeNeedleEl = document.getElementById('gaugeNeedle')

    if (gaugeValueEl && gaugeNeedleEl) {
      // Clamp angle between -90 and 90
      const clampedAngle = Math.max(-90, Math.min(90, newAngle))

      // Calculate gauge value (0 to 251.2, where 125.6 is center)
      const gaugeValue = 125.6 - (clampedAngle / 90) * 125.6
      gaugeValueEl.style.strokeDashoffset = gaugeValue

      // Rotate needle
      gaugeNeedleEl.style.transform = `rotate(${clampedAngle}deg)`
    }
  },
)

// Threshold settings update
watch(
  () => thresholdSettings,
  (newSettings) => {
    if (!connected.value || !thresholdConfigListener.value) return

    const converted = Object.fromEntries(
      Object.entries(newSettings).map(([key, value]) => {
        if (key.startsWith('bev')) {
          value = value / 100 // percentage to decimal
        }
        return [
          key.replace('_', '.'), // only first underscore
          parseFloat(value),
        ]
      }),
    )

    // Replace first underscore with dot
    const configString = JSON.stringify(converted)

    thresholdConfigListener.value.publish(
      new window.ROSLIB.Message({
        data: configString,
      }),
    )
  },
  { deep: true },
)

// Manual control
watch(
  () => manualControl.value,
  (isEnabled) => {
    if (!connected.value || !manualControlListener.value) return

    manualControlListener.value.publish(
      new window.ROSLIB.Message({
        data: isEnabled,
      }),
    )

    if (isEnabled) {
      startPolling()
      log('Manual control enabled.', 'info')
      showNotification('Manual control enabled. Robot stopped.', 'info')
    } else {
      stopPolling()
      log('Manual control disabled.', 'info')
      showNotification('Manual control disabled. Robot started.', 'info')
    }
  },
)

onMounted(() => {
  showNotification('Base Station Initialized.', 'success')
  log('Base Station Initialized. Ready to connect.')

  // Auto-connect to ROS bridge
  connectROS()
})

// Gamepad
const gamepad = computed(() => gamepads.value.find((g) => g.mapping === 'standard'))
const animationFrameId = ref(null) // Untuk menyimpan ID loop agar bisa di-stop
const DEADZONE = 0.1 // Untuk mengabaikan getaran kecil/drift pada joystick
onConnected((index) => {
  showNotification(`Gamepad connected: ${gamepads.value[index].id}`, 'info')
})

onDisconnected((_) => {
  showNotification(`Gamepad disconnected`, 'info')
})

/**
 * Loop utama untuk membaca data joystick.
 * Dijalankan 60x per detik menggunakan requestAnimationFrame.
 */
function pollGamepad() {
  // 1. Cek kondisi untuk melanjutkan loop
  // Jika mode manual mati, atau koneksi ROS putus, atau gamepad tidak ada, hentikan loop.
  if (!manualControl.value || !connected.value || !gamepad.value) {
    stopPolling() // Panggil fungsi stop untuk keamanan
    return
  }

  // 2. Baca nilai Sumbu (Axes)
  // [0] = Sumbu X Stik Kiri (-1.0 Kiri, 1.0 Kanan)
  // [1] = Sumbu Y Stik Kiri (-1.0 Atas, 1.0 Bawah)
  const rawX = gamepad.value.axes[0]
  const rawY = gamepad.value.axes[1]

  // 3. Terapkan Deadzone
  // Jika nilai absolutnya lebih kecil dari DEADZONE, anggap saja 0.
  const x = Math.abs(rawX) > DEADZONE ? rawX : 0.0
  const y = Math.abs(rawY) > DEADZONE ? rawY : 0.0

  // 4. Hitung Perintah
  // Sumbu Y terbalik (Atas = -1.0). Kita balik agar Atas (maju) = 1.0.
  const targetSpeed = y * -1.0 // Sekarang rentangnya -1.0 (mundur) s/d 1.0 (maju)
  // const targetSpeed = Math.abs(y * 1.0) // Hanya maju (0.0 s/d 1.0)

  // Sumbu X kita petakan ke -90 s/d 90 derajat
  const targetAngle = x * 90.0

  // 5. Publikasikan ke ROS (jika listener sudah siap)
  if (decisionListener.value) {
    decisionListener.value.publish(
      new window.ROSLIB.Message({
        data: JSON.stringify({
          type: 'command', // Tipe perintah manual
          speed: targetSpeed,
          turn_angle: targetAngle,
        }),
      }),
    )
  }

  // 6. Lanjutkan ke frame berikutnya
  animationFrameId.value = requestAnimationFrame(pollGamepad)
}

/**
 * Memulai loop polling gamepad.
 */
function startPolling() {
  if (animationFrameId.value) return // Loop sudah berjalan
  log('Starting gamepad polling...', 'info')
  animationFrameId.value = requestAnimationFrame(pollGamepad)
}

/**
 * Menghentikan loop polling gamepad dan mengirim perintah STOP.
 */
function stopPolling() {
  if (!animationFrameId.value) return // Loop sudah berhenti

  cancelAnimationFrame(animationFrameId.value)
  animationFrameId.value = null
  log('Gamepad polling stopped.', 'info')

  // PENTING: Kirim perintah STOP (0, 0) saat polling berhenti
  // agar robot tidak terus bergerak dengan perintah terakhir.
  if (decisionListener.value) {
    decisionListener.value.publish(
      new window.ROSLIB.Message({
        data: JSON.stringify({
          type: 'command',
          speed: 0.0,
          turn_angle: 0.0,
        }),
      }),
    )
  }
}

// Tambahkan watcher baru ini untuk menangani
// jika gamepad di-cabut/pasang SAAT mode manual aktif
watch(
  () => gamepad.value,
  (gp) => {
    if (manualControl.value) {
      if (gp) {
        startPolling() // Gamepad terdeteksi, mulai polling
      } else {
        stopPolling() // Gamepad terputus, hentikan polling
      }
    }
  },
)
</script>

<template>
  <nav class="navbar fixed-top navbar-expand-lg bg-body-tertiary">
    <div class="container d-flex justify-content-center">
      <router-link to="/" class="navbar-brand">
        <img src="/logo-dark.png" alt="NasgorGoreng" style="height: 40px" />
      </router-link>
    </div>
  </nav>
  <nav class="navbar fixed-bottom navbar-expand-lg bg-body-tertiary">
    <div class="container align-items-center">
      <div v-if="connected">
        <span class="status-indicator status-connected" id="connectionStatus"></span>
        <span class="small" id="connectionText" @click="manualControl = !manualControl"
          >Connected ({{ manualControl ? 'Manual control' : 'Autonomous' }}, click to switch)</span
        >
      </div>
      <div v-else>
        <span class="status-indicator status-disconnected" id="connectionStatus"></span>
        <span class="small" id="connectionText">Disconnected</span>
      </div>
      <div class="d-flex" role="search">
        <button
          type="button"
          class="btn btn-light me-3"
          id="manualControlBtn"
          data-bs-toggle="modal"
          data-bs-target="#manualControl"
          v-if="connected"
        >
          <i class="bi bi-controller"></i>
        </button>
        <button
          type="button"
          class="btn btn-warning me-3"
          data-bs-toggle="modal"
          data-bs-target="#configureConnection"
        >
          <i class="bi bi-gear-fill"></i>
        </button>

        <div v-if="connected">
          <button
            type="button"
            id="startRobotBtn"
            class="btn btn-success"
            v-if="manualControl"
            @click="manualControl = !manualControl"
          >
            <i class="bi bi-play-fill"></i>
          </button>
          <button
            type="button"
            id="stopRobotBtn"
            class="btn btn-danger"
            v-else
            @click="manualControl = !manualControl"
          >
            <i class="bi bi-stop-fill"></i>
          </button>
        </div>
      </div>
    </div>
  </nav>

  <main class="container py-5 my-5">
    <!-- Connection Settings -->
    <div
      class="modal fade"
      id="configureConnection"
      tabindex="-1"
      aria-labelledby="configureConnectionLabel"
      aria-hidden="true"
    >
      <div class="modal-dialog modal-dialog-centered modal-lg">
        <div class="modal-content">
          <div class="modal-header">
            <h1 class="modal-title fs-5" id="configureConnectionLabel">Connection Settings</h1>
            <button
              type="button"
              class="btn-close"
              data-bs-dismiss="modal"
              aria-label="Close"
            ></button>
          </div>
          <div class="modal-body">
            <div class="row g-3 mb-3">
              <div class="col-md-6">
                <label for="connectionType" class="form-label">Connection Type</label>
                <select id="connectionType" class="form-select" disabled>
                  <option value="websocket">Standard WebSocket</option>
                  <option value="ros" selected>ROS (roslibjs)</option>
                </select>
              </div>
              <div class="col-md-6">
                <label for="serverUrl" class="form-label">Server URL</label>
                <input
                  type="text"
                  id="serverUrl"
                  class="form-control"
                  placeholder="ws://localhost:9090"
                  v-model="serverURL"
                />
              </div>
            </div>

            <div id="rosSettings" class="mb-4">
              <div class="row g-3">
                <div class="col-md-6">
                  <label for="rosTopicRaw" class="form-label">Image Raw Topic</label>
                  <input
                    type="text"
                    id="rosTopicRaw"
                    class="form-control"
                    placeholder="/camera/image_raw/compressed"
                    v-model="imageRawTopic"
                  />
                </div>
                <div class="col-md-6">
                  <label for="rosTopicProcessed" class="form-label">Image Processed Topic</label>
                  <input
                    type="text"
                    id="rosTopicProcessed"
                    class="form-control"
                    placeholder="/camera/image_processed/compressed"
                    v-model="imageProcessedTopic"
                  />
                </div>
                <div class="col-md-6">
                  <label for="rosTopicTelemetry" class="form-label">Telemetry Topic</label>
                  <input
                    type="text"
                    id="rosTopicTelemetry"
                    class="form-control"
                    placeholder="/vehicle/telemetry"
                    v-model="telemetryTopic"
                  />
                </div>
                <div class="col-md-6">
                  <label for="rosTopicSensors" class="form-label">Sensors Topic</label>
                  <input
                    type="text"
                    id="rosTopicSensors"
                    class="form-control"
                    placeholder="/sensors"
                    v-model="sensorsTopic"
                  />
                </div>
                <div class="col-md-6">
                  <label for="rosTopicManualControl" class="form-label">Manual Control Topic</label>
                  <input
                    type="text"
                    id="rosTopicManualControl"
                    class="form-control"
                    placeholder="/manual_control"
                    v-model="manualControlTopic"
                  />
                </div>
                <div class="col-md-6">
                  <label for="rosThresholdConfig" class="form-label">Threshold Config Topic</label>
                  <input
                    type="text"
                    id="rosThresholdConfig"
                    class="form-control"
                    placeholder="/threshold_config"
                    v-model="thresholdConfigTopic"
                  />
                </div>
                <div class="col-md-6">
                  <label for="rosTopicDecision" class="form-label">Decision Topic</label>
                  <input
                    type="text"
                    id="rosTopicDecision"
                    class="form-control"
                    placeholder="/decision"
                    v-model="decisionTopic"
                  />
                </div>
              </div>
            </div>

            <div class="d-flex gap-2">
              <button
                id="connectBtn"
                class="btn btn-light"
                @click="connectROS"
                :disabled="connected"
              >
                Connect
              </button>
              <button
                id="disconnectBtn"
                class="btn btn-danger"
                @click="disconnectROS"
                :disabled="!connected"
              >
                Disconnect
              </button>
              <button
                id="logoutBtn"
                class="btn btn-warning"
                data-bs-dismiss="modal"
                @click="logout"
              >
                Logout from Base Station
              </button>
            </div>
          </div>
          <div class="modal-footer">
            <button type="button" class="btn btn-secondary" data-bs-dismiss="modal">Close</button>
          </div>
        </div>
      </div>
    </div>

    <!-- Manual Control Settings -->
    <div
      class="modal fade"
      id="manualControl"
      tabindex="-1"
      aria-labelledby="manualControlLabel"
      aria-hidden="true"
    >
      <div class="modal-dialog modal-dialog-centered modal-lg">
        <div class="modal-content">
          <div class="modal-header">
            <h1 class="modal-title fs-5" id="manualControlLabel">Manual Control</h1>
            <button
              type="button"
              class="btn-close"
              data-bs-dismiss="modal"
              aria-label="Close"
            ></button>
          </div>
          <div class="modal-body">
            <div class="row g-3 mb-3">
              <div class="col-md-6">
                <div class="form-check form-switch">
                  <input
                    class="form-check-input"
                    type="checkbox"
                    role="switch"
                    id="switchCheckDefault"
                    v-model="manualControl"
                  />
                  <label class="form-check-label" for="switchCheckDefault"
                    >Enable manual control</label
                  >
                </div>
              </div>
            </div>

            <div id="manualControlSettings" class="mb-4" v-if="manualControl">
              <div class="row g-3">
                <h5>Gamepad Status</h5>
                <div class="col-6" v-if="gamepad">
                  <p>Connected: {{ gamepad.id }}</p>

                  <Gamepad v-for="gamepad in gamepads" :key="gamepad.id" :gamepad="gamepad" />
                </div>
                <div class="col" v-else>
                  <p>
                    <i
                      >No gamepad connected. Please connect it and press any button to wake it
                      up.</i
                    >
                  </p>
                </div>
              </div>
            </div>
          </div>
          <div class="modal-footer">
            <button type="button" class="btn btn-secondary" data-bs-dismiss="modal">Close</button>
          </div>
        </div>
      </div>
    </div>

    <div v-if="connected">
      <div class="row g-4 mb-4">
        <div class="col-lg-6">
          <div class="card">
            <div class="card-body">
              <h3 class="h6 card-title mb-3">Raw Camera Feed</h3>
              <div class="video-container">
                <canvas ref="rawCanvasRef"></canvas>
                <div
                  class="video-placeholder text-center"
                  id="rawPlaceholder"
                  v-if="!imageMetadata.raw.resolution"
                >
                  Waiting for video stream...
                </div>
              </div>
              <div class="mt-3 d-flex align-items-center justify-content-between small text-muted">
                <span
                  >FPS: <span id="rawFps">{{ imageMetadata.raw.fps }}</span></span
                >
                <span
                  >Resolution:
                  <span id="rawResolution">{{ imageMetadata.raw.resolution }}</span></span
                >
              </div>
            </div>
          </div>
        </div>

        <div class="col-lg-6">
          <div class="card">
            <div class="card-body">
              <h3 class="h6 card-title mb-3">Processed Image (Threshold)</h3>
              <div class="video-container">
                <canvas ref="processedCanvasRef"></canvas>
                <div
                  class="video-placeholder"
                  id="processedPlaceholder"
                  v-if="!imageMetadata.processed.resolution"
                >
                  Waiting for video stream...
                </div>
              </div>
              <div class="mt-3 d-flex align-items-center justify-content-between small text-muted">
                <span
                  >FPS: <span id="processedFps">{{ imageMetadata.processed.fps }}</span></span
                >
                <span
                  >Resolution:
                  <span id="processedResolution">{{
                    imageMetadata.processed.resolution
                  }}</span></span
                >
              </div>
            </div>
          </div>
        </div>
      </div>

      <div class="row g-4 mb-4">
        <div class="accordion" id="thresholdSettingsAccordion">
          <div class="accordion-item">
            <h2 class="accordion-header">
              <button
                class="accordion-button"
                type="button"
                data-bs-toggle="collapse"
                data-bs-target="#thresholdSettings"
                aria-expanded="true"
                aria-controls="thresholdSettings"
              >
                <span>
                  <i class="bi bi-sliders2"></i>
                  Threshold Settings
                </span>
              </button>
            </h2>
            <div
              id="thresholdSettings"
              class="accordion-collapse collapse show"
              data-bs-parent="#thresholdSettingsAccordion"
            >
              <div class="accordion-body">
                <div class="row">
                  <div class="col-lg-12 col-md-12">
                    <label for="cntrLeftLaneLimitCenter" class="form-label"
                      >Contour Left Lane Limit Center (center_x -
                      {{ thresholdSettings.cntr_left_lane_limit_center }})
                    </label>
                    <input
                      type="range"
                      class="form-range"
                      min="0"
                      max="255"
                      id="cntrLeftLaneLimitCenter"
                      v-model="thresholdSettings.cntr_left_lane_limit_center"
                    />
                  </div>

                  <div class="col-lg-6 col-md-12">
                    <label for="cannyLowThreshold" class="form-label"
                      >Canny Low Threshold: {{ thresholdSettings.canny_low_threshold }}</label
                    >
                    <input
                      type="range"
                      class="form-range"
                      min="0"
                      max="255"
                      id="cannyLowThreshold"
                      v-model="thresholdSettings.canny_low_threshold"
                    />
                  </div>

                  <div class="col-lg-6 col-md-12">
                    <label for="cannyHighThreshold" class="form-label"
                      >Canny High Threshold: {{ thresholdSettings.canny_high_threshold }}</label
                    >
                    <input
                      type="range"
                      class="form-range"
                      min="0"
                      max="255"
                      id="cannyHighThreshold"
                      v-model="thresholdSettings.canny_high_threshold"
                    />
                  </div>

                  <div class="col-lg-6 col-md-12">
                    <label for="bevTopYLeftPercent" class="form-label"
                      >BEV Top Y Left Percent:
                      {{ thresholdSettings.bev_top_y_left_percent }}%</label
                    >
                    <input
                      type="range"
                      class="form-range"
                      min="0"
                      max="100"
                      id="bevTopYLeftPercent"
                      v-model="thresholdSettings.bev_top_y_left_percent"
                    />
                  </div>

                  <div class="col-lg-6 col-md-12">
                    <label for="bevTopYRightPercent" class="form-label"
                      >BEV Top Y Right Percent:
                      {{ thresholdSettings.bev_top_y_right_percent }}%</label
                    >
                    <input
                      type="range"
                      class="form-range"
                      min="0"
                      max="100"
                      id="bevTopYRightPercent"
                      v-model="thresholdSettings.bev_top_y_right_percent"
                    />
                  </div>

                  <div class="col-lg-6 col-md-12">
                    <label for="bevTopXLeftPercent" class="form-label"
                      >BEV Top X Left Percent:
                      {{ thresholdSettings.bev_top_x_left_percent }}%</label
                    >
                    <input
                      type="range"
                      class="form-range"
                      min="0"
                      max="100"
                      id="bevTopXLeftPercent"
                      v-model="thresholdSettings.bev_top_x_left_percent"
                    />
                  </div>

                  <div class="col-lg-6 col-md-12">
                    <label for="bevTopXRIghtPercent" class="form-label"
                      >BEV Top X Right Percent:
                      {{ thresholdSettings.bev_top_x_right_percent }}%</label
                    >
                    <input
                      type="range"
                      class="form-range"
                      min="0"
                      max="100"
                      id="bevTopXRIghtPercent"
                      v-model="thresholdSettings.bev_top_x_right_percent"
                    />
                  </div>

                  <div class="col-lg-6 col-md-12">
                    <label for="bevBottomXLeftPercent" class="form-label"
                      >BEV Bottom X Left Percent:
                      {{ thresholdSettings.bev_bottom_x_left_percent }}%</label
                    >
                    <input
                      type="range"
                      class="form-range"
                      min="0"
                      max="100"
                      id="bevBottomXLeftPercent"
                      v-model="thresholdSettings.bev_bottom_x_left_percent"
                    />
                  </div>

                  <div class="col-lg-6 col-md-12">
                    <label for="bevBottomXRightPercent" class="form-label"
                      >BEV Top X Right Percent:
                      {{ thresholdSettings.bev_bottom_x_right_percent }}%</label
                    >
                    <input
                      type="range"
                      class="form-range"
                      min="0"
                      max="200"
                      id="bevBottomXRightPercent"
                      v-model="thresholdSettings.bev_bottom_x_right_percent"
                    />
                  </div>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>

      <div class="row g-4 mb-4">
        <div class="col-lg-4 col-md-12">
          <div class="card">
            <div class="card-body">
              <h3 class="h6 card-title mb-4">Vehicle Information</h3>
              <div class="steering-gauge">
                <svg width="200" height="100" viewBox="0 0 200 100">
                  <path class="gauge-arc" d="M 20 100 A 80 80 0 0 1 180 100" />
                  <path
                    class="gauge-value"
                    id="gaugeValue"
                    d="M 20 100 A 80 80 0 0 1 180 100"
                    stroke-dasharray="251.2"
                    stroke-dashoffset="125.6"
                  />
                  <line class="gauge-needle" id="gaugeNeedle" x1="100" y1="100" x2="100" y2="30" />
                </svg>
              </div>
              <div class="text-center mt-2">
                <div class="h2 fw-bold" id="steeringValue">
                  {{ telemetry.steering_angle.toPrecision(2) }}°
                </div>
                <div class="small text-muted">Turn Angle</div>
              </div>
              <div class="d-flex flex-column gap-3">
                <div>
                  <div class="small text-muted mb-1">Speed</div>
                  <div class="h4" id="speed">{{ telemetry.speed }}</div>
                </div>
                <div>
                  <div class="small text-muted mb-1">Lane Status</div>
                  <span class="badge text-bg-success" id="laneStatus">{{
                    telemetry.laneStatus
                  }}</span>
                </div>
                <div>
                  <div class="small text-muted mb-1">Obstacle Detected</div>
                  <span class="badge text-bg-success" id="obstacleStatus">{{
                    telemetry.obstacleDetected
                  }}</span>
                </div>
                <div>
                  <div class="small text-muted mb-1">Obstacle Distance (cm)</div>
                  <div class="h4" id="obstacleDistance">{{ telemetry.obstacleDistance }} cm</div>
                </div>
              </div>
            </div>
          </div>
        </div>
        <div class="col-lg-8 col-md-12">
          <div class="card">
            <div class="card-body">
              <h3 class="h6 card-title mb-3">System Logs</h3>
              <div class="log-container" id="logContainer">
                <div v-for="(item, index) in logs" :key="index" class="log-entry">
                  <span class="log-timestamp">[{{ item.timestamp }}]</span>
                  <span :class="'log-' + item.type">{{ item.message }}</span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
    <div v-else class="my-5">
      <div class="text-center">
        <h3 class="mb-4">Not connected to ROS2 bridge.</h3>
        <button
          type="button"
          class="btn btn-warning"
          data-bs-toggle="modal"
          data-bs-target="#configureConnection"
        >
          <i class="bi bi-gear-fill"></i>
          Configure Connection
        </button>
      </div>
      <div class="row g-4 my-4">
        <div class="col-12">
          <div class="card">
            <div class="card-body">
              <h3 class="h6 card-title mb-3">System Logs</h3>
              <div class="log-container" id="logContainer">
                <div v-for="(item, index) in logs" :key="index" class="log-entry">
                  <span class="log-timestamp">[{{ item.timestamp }}]</span>
                  <span :class="'log-' + item.type">{{ item.message }}</span>
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </main>
</template>
