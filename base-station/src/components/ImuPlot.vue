<template>
    <div class="plot" ref="root"></div>
</template>

<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref } from 'vue'
import uPlot from 'uplot'
import 'uplot/dist/uPlot.min.css'
import { useTelemetryStore } from '../stores/telemetry'
import { storeToRefs } from 'pinia'
import { watch } from 'vue'

const store = useTelemetryStore()
const { version } = storeToRefs(store)

watch(version, () => {
    plot?.setData(buildData())
})

const root = ref<HTMLDivElement | null>(null)

let plot: uPlot | null = null
let raf = 0

function buildData(): uPlot.AlignedData {
    const w = store.ring.window()

    if (w.x.length < 2) {
        return [new Float64Array(), new Float64Array(), new Float64Array(), new Float64Array()]
    }

    const ax0 = w.ax[0]
    const ay0 = w.ay[0]
    const az0 = w.az[0]

    const x = Float64Array.from(w.x)
    const ax = Float64Array.from(w.ax, v => v - ax0)
    const ay = Float64Array.from(w.ay, v => v - ay0)
    const az = Float64Array.from(w.az, v => v - az0)

    return [x, ax, ay, az]
}


onMounted(() => {
    plot = new uPlot({
        width: root.value!.clientWidth,
        height: 260,
        scales: {
            x: { time: false }, // ✅ plus de 1970
        },
        series: [
            {},
            { label: 'ax', stroke: '#d00', width: 2 },
            { label: 'ay', stroke: '#0a0', width: 2 },
            { label: 'az', stroke: '#00a', width: 2 },
        ]


    }, buildData(), root.value!)
})

onBeforeUnmount(() => {
    cancelAnimationFrame(raf)
    plot?.destroy()
    plot = null
})

watch(() => store.version, () => {
    plot?.setData(buildData())
})


</script>

<style scoped>
.plot :deep(.uplot) {
  background: white;
  color: black;
}

.plot :deep(.u-label),
.plot :deep(.u-title),
.plot :deep(.u-legend) {
  color: black;
}

</style>
