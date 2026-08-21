// FPS Counters
class FPSCounter {
  constructor() {
    this.frameCount = 0
    this.lastTime = performance.now()
    this.fps = 0
  }

  tick() {
    this.frameCount++
    const currentTime = performance.now()
    const delta = currentTime - this.lastTime

    if (delta >= 1000) {
      this.fps = this.frameCount
      this.frameCount = 0
      this.lastTime = currentTime
    }
  }
}

export default FPSCounter
