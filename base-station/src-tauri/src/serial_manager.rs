use std::{sync::Mutex, time::Duration};

use tauri::ipc::Channel;
use tokio_util::sync::CancellationToken;

use crate::{
    processors::{
        attitude::AttitudeProcessor, gyro_setpoint::GyroSetpointProcessor, imu::ImuProcessor,
        Processor,
    },
    protocol::BusFrameParser,
    telemetry::{PortInfo, Stats, TelemetryEvent},
};

#[derive(Default)]
pub struct StreamState {
    cancel: Option<CancellationToken>,
}

#[tauri::command]
pub fn list_ports() -> Result<Vec<PortInfo>, String> {
    let ports = tokio_serial::available_ports().map_err(|e| e.to_string())?;
    Ok(ports
        .into_iter()
        .map(|p| PortInfo { name: p.port_name })
        .collect())
}

#[tauri::command]
pub fn stop_telemetry_stream(state: tauri::State<'_, Mutex<StreamState>>) {
    if let Some(c) = state.lock().unwrap().cancel.take() {
        c.cancel();
    }
}

#[tauri::command]
pub fn start_telemetry_stream(
    port: String,
    baud: u32,
    ui_hz: u32,
    on_event: Channel<TelemetryEvent>,
    state: tauri::State<'_, Mutex<StreamState>>,
) -> Result<(), String> {
    stop_telemetry_stream(state.clone());

    let cancel = CancellationToken::new();
    state.lock().unwrap().cancel = Some(cancel.clone());

    tauri::async_runtime::spawn(async move {
        if let Err(e) = run_stream(port, baud, ui_hz, on_event, cancel).await {
            eprintln!("[telemetry_stream] error: {e}");
        }
    });

    Ok(())
}

async fn run_stream(
    port: String,
    baud: u32,
    ui_hz: u32,
    on_event: Channel<TelemetryEvent>,
    cancel: CancellationToken,
) -> Result<(), String> {
    use tokio::io::AsyncReadExt;

    let builder = tokio_serial::new(port, baud);
    let mut serial = tokio_serial::SerialStream::open(&builder).map_err(|e| e.to_string())?;

    let mut parser = BusFrameParser::default();
    let mut buf = [0u8; 1024];

    // Plug processors here. They do not know each other.
    let mut procs: Vec<Box<dyn Processor>> = vec![
        Box::new(ImuProcessor::new()),
        Box::new(AttitudeProcessor::new("rad")), // or "deg"
        Box::new(GyroSetpointProcessor::new()),  // or "deg"
    ];

    let mut frames_total: u64 = 0;

    let hz = ui_hz.max(1).min(240);
    let mut tick = tokio::time::interval(Duration::from_millis(1000 / hz as u64));
    let mut stat_tick = tokio::time::interval(Duration::from_millis(500));

    loop {
        tokio::select! {
            _ = cancel.cancelled() => break,

            _ = tick.tick() => {
                for p in procs.iter_mut() {
                    if let Some(ev) = p.on_tick() {
                        let _ = on_event.send(ev);
                    }
                }
            }

            _ = stat_tick.tick() => {
                let mut s = Stats {
                    frames_total,
                    imu_ok: 0,
                    imu_bad_len: 0,
                    att_ok: 0,
                    att_bad_len: 0,
                    gyro_setpoint_ok: 0,
                    gyro_setpoint_bad_len: 0,
                };
                for p in procs.iter() {
                    p.add_stats(&mut s);
                }
                let _ = on_event.send(TelemetryEvent::Stats(s));
            }

            read_res = serial.read(&mut buf) => {
                let n = read_res.map_err(|e| e.to_string())?;
                if n == 0 { continue; }

                let frames = parser.feed(&buf[..n]);
                for f in frames {
                    frames_total += 1;

                    let pc_us: u64 = std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap()
                        .as_micros() as u64;

                    for p in procs.iter_mut() {
                        p.on_frame(&f, pc_us);
                    }
                }
            }
        }
    }

    Ok(())
}
