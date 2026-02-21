<template>
    <div style="padding: 16px; display: grid; gap: 12px; max-width: 900px">
        <div
            style="
                display: flex;
                gap: 8px;
                flex-wrap: wrap;
                align-items: center;
            "
        >
            <select v-model="selectedPort">
                <option v-for="p in ports" :key="p.name" :value="p.name">
                    {{ p.name }}
                </option>
            </select>

            <input v-model.number="baud" type="number" style="width: 120px" />
            <input v-model.number="uiHz" type="number" style="width: 90px" />

            <button @click="start">Start</button>
            <button @click="stop">Stop</button>
            <button @click="refreshPorts">Refresh ports</button>
        </div>

        <div>
            <AccelPlot />
        </div>

        <div>
            <GyroPlot />
        </div>
        <div>
            <GyroSetpointPlot />
        </div>
        <div>
            <Attitude3D />
        </div>

        <pre>{{ store.stats }}</pre>
    </div>
</template>

<script setup lang="ts">
import { onMounted, ref } from "vue";
import { invoke, Channel } from "@tauri-apps/api/core";
import AccelPlot from "./components/AccelPlot.vue";
import GyroPlot from "./components/GyroPlot.vue";
import GyroSetpointPlot from "./components/GyroSetpointPlot.vue";
import { useTelemetryStore } from "./stores/telemetry";
import Attitude3D from "./components/Attitude3D.vue";
import { startTelemetryStream } from "./services/telemetryStream";
type PortInfo = { name: string };

type TelemetryEvent =
    | { event: "imuRawChunk"; data: any }
    | {
          event: "stats";
          data: { framesTotal: number; imuOk: number; imuBadLen: number };
      };

const store = useTelemetryStore();

const ports = ref<PortInfo[]>([]);
const selectedPort = ref("COM3");
const baud = ref(115200);
const uiHz = ref(60);

let ch: Channel<TelemetryEvent> | null = null;

let disconnectTelemetry: (() => Promise<void>) | null = null;

async function refreshPorts() {
    ports.value = await invoke<PortInfo[]>("list_ports");
    if (
        ports.value.length &&
        !ports.value.find((p) => p.name === selectedPort.value)
    ) {
        selectedPort.value = ports.value[0].name;
    }
}

async function start() {
    ch = new Channel<TelemetryEvent>();
    ch.onmessage = (msg) => {
        if (msg.event === "imuRawChunk") {
            store.pushImuChunk(msg.data);
        } else if (msg.event === "stats") {
            Object.assign(store.stats, msg.data);
        }
    };
    disconnectTelemetry = await startTelemetryStream(
        selectedPort.value,
        baud.value,
        uiHz.value,
    );
}

async function stop() {
    if (disconnectTelemetry) {
        disconnectTelemetry();
    }
    ch = null;
}

onMounted(refreshPorts);
</script>
