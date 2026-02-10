import { defineStore } from 'pinia'
import { computed, markRaw, ref } from 'vue'

export type ImuChunk = {
  // MCU timestamp (debug / log), pas utilisé pour l'axe X UI
  tUs: number[]
  // PC timestamp en microsecondes (monotone-ish), utilisé pour l'axe X UI
  pcUs: number[]

  ax: number[]; ay: number[]; az: number[]
  gx: number[]; gy: number[]; gz: number[]
}

export type Stats = {
  framesTotal: number
  imuOk: number
  imuBadLen: number
}

class Ring {
  private cap: number
  private lastT = -Infinity // dernière valeur pcUs (pour monotonic)

  // on stocke pcUs ici
  t: number[] = []
  ax: number[] = []
  ay: number[] = []
  az: number[] = []

  constructor(capacity: number) {
    this.cap = capacity
  }

  pushChunk(c: ImuChunk) {
    const n = Math.min(c.pcUs.length, c.ax.length, c.ay.length, c.az.length)
    if (n <= 0) return

    for (let i = 0; i < n; i++) {
      let t = c.pcUs[i]

      // ✅ force strictement croissant (uPlot aime ça)
      if (t <= this.lastT) t = this.lastT + 1
      this.lastT = t

      this.t.push(t)
      this.ax.push(c.ax[i])
      this.ay.push(c.ay[i])
      this.az.push(c.az[i])
    }

    // trim
    const extra = this.t.length - this.cap
    if (extra > 0) {
      this.t.splice(0, extra)
      this.ax.splice(0, extra)
      this.ay.splice(0, extra)
      this.az.splice(0, extra)
    }
  }

  // fenêtre pour chart: retourne X en secondes relatives
  window() {
    if (this.t.length === 0) return { x: [], ax: [], ay: [], az: [] }

    const t0 = this.t[0]
    const x = this.t.map(v => (v - t0) * 1e-6) // microseconds -> seconds

    return { x, ax: this.ax, ay: this.ay, az: this.az }
  }

  latest() {
    const n = this.t.length
    if (n === 0) return null
    return {
      pcUs: this.t[n - 1],
      ax: this.ax[n - 1],
      ay: this.ay[n - 1],
      az: this.az[n - 1],
    }
  }
}

export const useTelemetryStore = defineStore('telemetry', () => {
  const ring = markRaw(new Ring(4000))

  // incrémenté à chaque chunk => le plot watch dessus
  const version = ref(0)

  const stats = ref<Stats>({ framesTotal: 0, imuOk: 0, imuBadLen: 0 })

  const latest = ref<{ pcUs: number; ax: number; ay: number; az: number } | null>(null)

  const lastRxMs = ref(0)
  const isStale = computed(() => Date.now() - lastRxMs.value > 500)

  function pushImuChunk(c: ImuChunk) {
    ring.pushChunk(c)
    latest.value = ring.latest()
    lastRxMs.value = Date.now()
    version.value++
  }

  function setStats(s: Partial<Stats>) {
    Object.assign(stats.value, s)
  }

  return { ring, version, stats, latest, lastRxMs, isStale, pushImuChunk, setStats }
})
