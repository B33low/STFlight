import { invoke, Channel } from "@tauri-apps/api/core"
import { useTelemetryStore, type TelemetryEvent } from "../stores/telemetry"

export async function startTelemetryStream(port: string, baud = 921600, uiHz = 60) {
  const store = useTelemetryStore()

  const ch = new Channel<TelemetryEvent>()
  ch.onmessage = (ev) => {
    store.ingestEvent(ev)
  }

  await invoke("start_telemetry_stream", {
    port,
    baud,
    uiHz: uiHz,
    onEvent: ch,
  })

  return async () => {
    await invoke("stop_telemetry_stream")
  }
}
