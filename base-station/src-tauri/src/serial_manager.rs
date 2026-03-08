use std::{sync::Mutex, time::Duration, io::Write};

use byteorder::{LittleEndian, WriteBytesExt};
use tauri::ipc::Channel;
use tokio::sync::mpsc;
use tokio_util::sync::CancellationToken;

use crate::{
    processors::{
        attitude::AttitudeProcessor, gyro_setpoint::GyroSetpointProcessor, imu::ImuProcessor,
        Processor,
    },
    protocol::{BusFrameParser, MsgKind, MsgType, frame_to_bytes},
    telemetry::{PortInfo, Stats, TelemetryEvent},
};

pub struct StreamState {
    cancel: Option<CancellationToken>,
    inject_tx: Option<mpsc::Sender<Vec<u8>>>,
}

impl Default for StreamState {
    fn default() -> Self {
        Self {
            cancel: None,
            inject_tx: None,
        }
    }
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
pub fn inject_gyro_setpoint(port: String, baud: u32, gx: i16, gy: i16, gz: i16, state: tauri::State<'_, Mutex<StreamState>>) -> Result<(), String> {
    eprintln!("[inject_gyro_setpoint] Attempting to inject: gx={}, gy={}, gz={} on {}@{}", gx, gy, gz, port, baud);
    
    // Pack the gyro setpoint into 6 bytes (little-endian)
    let mut payload = Vec::new();
    payload.write_i16::<LittleEndian>(gx).map_err(|e| {
        let err_msg = format!("Failed to write gx: {}", e);
        eprintln!("[inject_gyro_setpoint] {}", err_msg);
        err_msg
    })?;
    payload.write_i16::<LittleEndian>(gy).map_err(|e| {
        let err_msg = format!("Failed to write gy: {}", e);
        eprintln!("[inject_gyro_setpoint] {}", err_msg);
        err_msg
    })?;
    payload.write_i16::<LittleEndian>(gz).map_err(|e| {
        let err_msg = format!("Failed to write gz: {}", e);
        eprintln!("[inject_gyro_setpoint] {}", err_msg);
        err_msg
    })?;

    eprintln!("[inject_gyro_setpoint] Payload packed: {:?}", payload);

    // Create injection frame: ID_GYRO_SETPOINT_STATE = 5, BUS_KIND_STATE = 1
    const ID_GYRO_SETPOINT_STATE: u8 = 5;
    let frame_bytes = frame_to_bytes(MsgType::BusMsgInject, MsgKind::BusKindState, ID_GYRO_SETPOINT_STATE, &payload);
    
    eprintln!("[inject_gyro_setpoint] Frame bytes: {:02X?}", frame_bytes);

    // Try to send through the existing stream if available
    let stream_state = state.lock().unwrap();
    if let Some(tx) = &stream_state.inject_tx {
        eprintln!("[inject_gyro_setpoint] Sending through telemetry stream channel");
        tx.try_send(frame_bytes.clone()).map_err(|e| {
            let err_msg = format!("Failed to queue injection: {}", e);
            eprintln!("[inject_gyro_setpoint] {}", err_msg);
            err_msg
        })?;
        eprintln!("[inject_gyro_setpoint] Success!");
        Ok(())
    } else {
        eprintln!("[inject_gyro_setpoint] No active telemetry stream. Trying direct port access");
        drop(stream_state); // Release lock before opening port
        
        let mut port_handle = serialport::new(&port, baud)
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|e| {
                let err_msg = format!("Failed to open port {}: {} (make sure telemetry stream is running)", port, e);
                eprintln!("[inject_gyro_setpoint] {}", err_msg);
                err_msg
            })?;
        
        eprintln!("[inject_gyro_setpoint] Port opened, writing {} bytes", frame_bytes.len());
        
        port_handle.write_all(&frame_bytes).map_err(|e| {
            let err_msg = format!("Failed to write to port: {}", e);
            eprintln!("[inject_gyro_setpoint] {}", err_msg);
            err_msg
        })?;

        eprintln!("[inject_gyro_setpoint] Success!");
        Ok(())
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
    let (inject_tx, inject_rx) = mpsc::channel(16);
    
    {
        let mut st = state.lock().unwrap();
        st.cancel = Some(cancel.clone());
        st.inject_tx = Some(inject_tx);
    }

    tauri::async_runtime::spawn(async move {
        if let Err(e) = run_stream(port, baud, ui_hz, on_event, cancel, inject_rx).await {
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
    mut inject_rx: mpsc::Receiver<Vec<u8>>,
) -> Result<(), String> {
    use tokio::io::AsyncReadExt;
    use tokio::io::AsyncWriteExt;

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

            Some(injection_frame) = inject_rx.recv() => {
                eprintln!("[telemetry_stream] Sending injection frame: {:02X?}", injection_frame);
                use tokio::io::AsyncWriteExt;
                if let Err(e) = AsyncWriteExt::write_all(&mut serial, &injection_frame).await {
                    eprintln!("[telemetry_stream] Failed to write injection frame: {}", e);
                }
            }
        }
    }

    Ok(())
}
