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
const { imuVersion } = storeToRefs(store)

watch(imuVersion, () => {
    plot?.setData(buildData())
})
const WINDOW_S = 10.0;
const root = ref<HTMLDivElement | null>(null)

let plot: uPlot | null = null
let raf = 0

function buildData(): uPlot.AlignedData {
    const w = store.imuRing.window(4096)

    if (w.x.length < 2) {
        return [new Float64Array(), new Float64Array(), new Float64Array(), new Float64Array()]
    }

    const ax0 = w.ax[0]
    const ay0 = w.ay[0]
    const az0 = w.az[0]
    const tLast = w.x[w.x.length - 1]
    const x = Float64Array.from(w.x, v => v-tLast )
    const ax = Float64Array.from(w.ax, v => v )
    const ay = Float64Array.from(w.ay, v => v )
    const az = Float64Array.from(w.az, v => v )

    return [x, ax, ay, az]
}


onMounted(() => {
    plot = new uPlot({
        width: root.value!.clientWidth,
        height: 260,
        scales: {
            x: {
        time: false,
        range: () => [-WINDOW_S, 0],   // always show [-W, 0]
      },
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

watch(() => store.imuVersion, () => {
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
