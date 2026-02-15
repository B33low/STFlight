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
    const w = store.imuRing.window(1024)

    if (w.x.length < 2) {
        return [new Float64Array(), new Float64Array(), new Float64Array(), new Float64Array()]
    }

    const gx0 = w.gx[0]
    const gy0 = w.gy[0]
    const gz0 = w.gz[0]
    const tLast = w.x[w.x.length - 1]
    const x = Float64Array.from(w.x, v => v-tLast )
    const gx = Float64Array.from(w.gx, v => v )
    const gy = Float64Array.from(w.gy, v => v )
    const gz = Float64Array.from(w.gz, v => v )

    return [x, gx, gy, gz]
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
            { label: 'gx', stroke: '#d00', width: 2 },
            { label: 'gy', stroke: '#0a0', width: 2 },
            { label: 'gz', stroke: '#00a', width: 2 },
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
