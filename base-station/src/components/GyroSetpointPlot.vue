<template>
    <div class="plot" ref="root"></div>
</template>

<script setup lang="ts">
import { onBeforeUnmount, onMounted, ref } from "vue";
import uPlot from "uplot";
import "uplot/dist/uPlot.min.css";
import { useTelemetryStore } from "../stores/telemetry";
import { storeToRefs } from "pinia";
import { watch } from "vue";

const store = useTelemetryStore();
const { gyroSetpointVersion } = storeToRefs(store);

watch(gyroSetpointVersion, () => {
    plot?.setData(buildData());
});
const WINDOW_S = 10.0;
const root = ref<HTMLDivElement | null>(null);

let plot: uPlot | null = null;
let raf = 0;

function buildData(): uPlot.AlignedData {
    const w = store.gyroSetpointRing.window(1024);

    if (w.x.length < 2) {
        return [
            new Float64Array(),
            new Float64Array(),
            new Float64Array(),
            new Float64Array(),
        ];
    }

    const tLast = w.x[w.x.length - 1];
    const x = Float64Array.from(w.x, (v) => v - tLast);
    const rx = Float64Array.from(w.rx, (v) => v);
    const ry = Float64Array.from(w.ry, (v) => v);
    const rz = Float64Array.from(w.rz, (v) => v);

    return [x, rx, ry, rz];
}

onMounted(() => {
    plot = new uPlot(
        {
            width: root.value!.clientWidth,
            height: 260,
            scales: {
                x: {
                    time: false,
                    range: () => [-WINDOW_S, 0], // always show [-W, 0]
                },
            },
            series: [
                {},
                { label: "rx", stroke: "#d00", width: 2 },
                { label: "ry", stroke: "#0a0", width: 2 },
                { label: "rz", stroke: "#00a", width: 2 },
            ],
        },
        buildData(),
        root.value!,
    );
});

onBeforeUnmount(() => {
    cancelAnimationFrame(raf);
    plot?.destroy();
    plot = null;
});

watch(
    () => store.gyroSetpointVersion,
    () => {
        plot?.setData(buildData());
    },
);
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
