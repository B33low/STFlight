// stores/telemetry.ts
import { defineStore } from "pinia"
import { computed, markRaw, ref, shallowRef } from "vue"

/** =======================
 *  Types coming from backend
 *  ======================= */

export type ImuChunk = {
    tUs: number[]
    pcUs: number[]
    ax: number[]; ay: number[]; az: number[]
    gx: number[]; gy: number[]; gz: number[]
}

export type GyroSetpointChunk = {
    pcUs: number[]
    tUs: number[]
    gx: number[]; gy: number[]; gz: number[]
}


// backend sends attitude as a chunk (arrays) -> we'll keep latest sample only
export type AttitudeChunk = {
    pcUs: number[]
    roll: number[]
    pitch: number[]
    yaw: number[]
    unit: "deg" | "rad"
}

export type AttitudeEulerSample = {
    pcUs: number
    roll: number
    pitch: number
    yaw: number
    unit?: "deg" | "rad"
}

export type Stats = {
    framesTotal: number
    imuOk: number
    imuBadLen: number
    attOk: number
    attBadLen: number
    gyroSetpointOk: number
    gyroSetpointBadLen: number
}

// This matches serde(tag="event", content="data", rename_all="camelCase")
export type TelemetryEvent =
    | { event: "imuRawChunk"; data: ImuChunk }
    | { event: "attitudeChunk"; data: AttitudeChunk }
    | { event: "gyroSetpointChunk"; data: GyroSetpointChunk }
    | { event: "stats"; data: Stats }

/** =======================
 *  IMU Ring
 *  ======================= */

class ImuRing {
    private cap: number
    private lastT = -Infinity

    t: number[] = []
    ax: number[] = []
    ay: number[] = []
    az: number[] = []
    gx: number[] = []
    gy: number[] = []
    gz: number[] = []

    constructor(capacity: number) {
        this.cap = capacity
    }

    pushChunk(c: ImuChunk) {
        const n = Math.min(
            c.pcUs.length,
            c.ax.length, c.ay.length, c.az.length,
            c.gx.length, c.gy.length, c.gz.length,
        )
        if (n <= 0) return

        for (let i = 0; i < n; i++) {
            let t = c.pcUs[i]
            if (t <= this.lastT) t = this.lastT + 1
            this.lastT = t

            this.t.push(t)
            this.ax.push(c.ax[i])
            this.ay.push(c.ay[i])
            this.az.push(c.az[i])
            this.gx.push(c.gx[i])
            this.gy.push(c.gy[i])
            this.gz.push(c.gz[i])
        }

        const extra = this.t.length - this.cap
        if (extra > 0) {
            this.t.splice(0, extra)
            this.ax.splice(0, extra)
            this.ay.splice(0, extra)
            this.az.splice(0, extra)
            this.gx.splice(0, extra)
            this.gy.splice(0, extra)
            this.gz.splice(0, extra)
        }
    }

    window(count: number, xSecMode: "fromFirst" | "nowRight" = "fromFirst") {
        const n = this.t.length
        if (n === 0) return { x: [], ax: [], ay: [], az: [], gx: [], gy: [], gz: [] }

        const start = Math.max(n - Math.floor(count), 0)
        const tSlice = this.t.slice(start)

        let x: number[]
        if (xSecMode === "fromFirst") {
            const t0 = tSlice[0]
            x = tSlice.map(v => (v - t0) * 1e-6)
        } else {
            const tLast = tSlice[tSlice.length - 1]
            x = tSlice.map(v => (v - tLast) * 1e-6) // <=0 with 0 at right
        }

        return {
            x,
            ax: this.ax.slice(start),
            ay: this.ay.slice(start),
            az: this.az.slice(start),
            gx: this.gx.slice(start),
            gy: this.gy.slice(start),
            gz: this.gz.slice(start),
        }
    }

    latest() {
        const n = this.t.length
        if (n === 0) return null
        return {
            pcUs: this.t[n - 1],
            ax: this.ax[n - 1],
            ay: this.ay[n - 1],
            az: this.az[n - 1],
            gx: this.gx[n - 1],
            gy: this.gy[n - 1],
            gz: this.gz[n - 1],
        }
    }
}

class GyroSetpointRing {
    private cap: number
    private lastT = -Infinity

    t: number[] = []
    gx: number[] = []
    gy: number[] = []
    gz: number[] = []

    constructor(capacity: number) {
        this.cap = capacity
    }

    pushChunk(c: GyroSetpointChunk) {
        const n = Math.min(
            c.pcUs.length,
            c.gx.length, c.gy.length, c.gz.length,
        )
        if (n <= 0) return

        for (let i = 0; i < n; i++) {
            let t = c.pcUs[i]
            if (t <= this.lastT) t = this.lastT + 1
            this.lastT = t

            this.t.push(t)
            this.gx.push(c.gx[i])
            this.gy.push(c.gy[i])
            this.gz.push(c.gz[i])
        }

        const extra = this.t.length - this.cap
        if (extra > 0) {
            this.t.splice(0, extra)
            this.gx.splice(0, extra)
            this.gy.splice(0, extra)
            this.gz.splice(0, extra)
        }
    }

    window(count: number, xSecMode: "fromFirst" | "nowRight" = "fromFirst") {
        const n = this.t.length
        if (n === 0) return { x: [], gx: [], gy: [], gz: [] }

        const start = Math.max(n - Math.floor(count), 0)
        const tSlice = this.t.slice(start)

        let x: number[]
        if (xSecMode === "fromFirst") {
            const t0 = tSlice[0]
            x = tSlice.map(v => (v - t0) * 1e-6)
        } else {
            const tLast = tSlice[tSlice.length - 1]
            x = tSlice.map(v => (v - tLast) * 1e-6) // <=0 with 0 at right
        }

        return {
            x,
            gx: this.gx.slice(start),
            gy: this.gy.slice(start),
            gz: this.gz.slice(start),
        }
    }

    latest() {
        const n = this.t.length
        if (n === 0) return null
        return {
            pcUs: this.t[n - 1],
            gx: this.gx[n - 1],
            gy: this.gy[n - 1],
            gz: this.gz[n - 1],
        }
    }
}


/** =======================
 *  Store
 *  ======================= */

export const useTelemetryStore = defineStore("telemetry", () => {
    const imuRing = markRaw(new ImuRing(4000))
    const gyroSetpointRing = markRaw(new GyroSetpointRing(4000))

    const imuVersion = ref(0)
    const attVersion = ref(0)
    const gyroSetpointVersion = ref(0)

    // Backward-compat alias if some components still watch "version"
    const version = imuVersion

    const stats = ref<Stats>({
        framesTotal: 0,
        imuOk: 0,
        imuBadLen: 0,
        attOk: 0,
        attBadLen: 0,
        gyroSetpointOk: 0,
        gyroSetpointBadLen: 0,
    })

    const latestImu = ref<{
        pcUs: number; ax: number; ay: number; az: number; gx: number; gy: number; gz: number
    } | null>(null)

    const latestGyroSetpoint = ref<{
        pcUs: number; gx: number; gy: number; gz: number
    } | null>(null)

    const attitudeEuler = shallowRef<AttitudeEulerSample | null>(null)

    const lastRxMs = ref(0)
    const isStale = computed(() => Date.now() - lastRxMs.value > 500)

    function pushImuChunk(c: ImuChunk) {
        imuRing.pushChunk(c)
        latestImu.value = imuRing.latest()
        lastRxMs.value = Date.now()
        imuVersion.value++
    }

    function pushGyroSetpointChunk(c: GyroSetpointChunk) {
        gyroSetpointRing.pushChunk(c)
        latestGyroSetpoint.value = gyroSetpointRing.latest()
        lastRxMs.value = Date.now()
        gyroSetpointVersion.value++
    }

    function pushAttitudeChunk(c: AttitudeChunk) {
        // keep latest sample only (good for 3D)
        const n = c.pcUs.length
        if (n <= 0) return
        const i = n - 1
        attitudeEuler.value = {
            pcUs: c.pcUs[i],
            roll: c.roll[i],
            pitch: c.pitch[i],
            yaw: c.yaw[i],
            unit: c.unit,
        }
        lastRxMs.value = Date.now()
        attVersion.value++
    }


    function ingestEvent(ev: TelemetryEvent) {
        switch (ev.event) {
            case "imuRawChunk":
                pushImuChunk(ev.data)
                break
            case "attitudeChunk":
                pushAttitudeChunk(ev.data)
                break
            case "gyroSetpointChunk":
                pushGyroSetpointChunk(ev.data)
                break
            case "stats":
                stats.value = ev.data
                break
        }
    }

    return {
        imuRing,
        gyroSetpointRing,
        imuVersion,
        attVersion,
        gyroSetpointVersion,
        version, // alias
        stats,
        latestImu,
        latestGyroSetpoint,
        attitudeEuler,
        lastRxMs,
        isStale,
        pushImuChunk,
        pushAttitudeChunk,
        pushGyroSetpointChunk,
        ingestEvent,
    }
})
