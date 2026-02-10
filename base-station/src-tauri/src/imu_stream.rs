// src-tauri/src/imu_stream.rs
use std::{io::Cursor, sync::Mutex, time::Duration};

use byteorder::{LittleEndian, ReadBytesExt};
use serde::Serialize;
use tauri::ipc::Channel;
use tokio_util::sync::CancellationToken;

use crate::protocol::{BusFrameParser, Frame};

const BUS_MSG_PUBLISH: u8 = 1;
const BUS_KIND_STREAM: u8 = 3;
const ID_IMU_RAW: u8 = 1;

// <hhhhhhhI => 7*i16 + u32 = 18 bytes
const IMU_SIZE: usize = 18;

#[derive(Debug, Clone, Copy)]
struct ImuRaw {
  ax: i16,
  ay: i16,
  az: i16,
  gx: i16,
  gy: i16,
  gz: i16,
  temp: i16,
  t_us: u32,
}

fn parse_imu_raw(payload: &[u8]) -> Option<ImuRaw> {
  if payload.len() != IMU_SIZE {
    return None;
  }
  let mut rdr = Cursor::new(payload);
  Some(ImuRaw {
    ax: rdr.read_i16::<LittleEndian>().ok()?,
    ay: rdr.read_i16::<LittleEndian>().ok()?,
    az: rdr.read_i16::<LittleEndian>().ok()?,
    gx: rdr.read_i16::<LittleEndian>().ok()?,
    gy: rdr.read_i16::<LittleEndian>().ok()?,
    gz: rdr.read_i16::<LittleEndian>().ok()?,
    temp: rdr.read_i16::<LittleEndian>().ok()?,
    t_us: rdr.read_u32::<LittleEndian>().ok()?,
  })
}

#[derive(Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PortInfo {
  pub name: String,
}

#[derive(Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct ImuChunk {
pub pc_us: Vec<u64>, 
  pub t_us: Vec<u32>,
  pub ax: Vec<i16>,
  pub ay: Vec<i16>,
  pub az: Vec<i16>,
  pub gx: Vec<i16>,
  pub gy: Vec<i16>,
  pub gz: Vec<i16>,
}

#[derive(Clone, serde::Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Stats {
  pub frames_total: u64,
  pub imu_ok: u64,
  pub imu_bad_len: u64,
}

#[derive(Clone, Serialize)]
#[serde(tag = "event", content = "data", rename_all = "camelCase")]
pub enum TelemetryEvent {
  ImuRawChunk(ImuChunk),
  Stats(Stats),
}

#[derive(Default)]
pub struct StreamState {
  cancel: Option<CancellationToken>,
}

#[tauri::command]
pub fn list_ports() -> Result<Vec<PortInfo>, String> {
  let ports = tokio_serial::available_ports().map_err(|e| e.to_string())?;
  Ok(ports.into_iter().map(|p| PortInfo { name: p.port_name }).collect())
}

#[tauri::command]
pub fn stop_imu_stream(state: tauri::State<'_, Mutex<StreamState>>) {
  if let Some(c) = state.lock().unwrap().cancel.take() {
    c.cancel();
  }
}

#[tauri::command]
pub fn start_imu_stream(
  port: String,
  baud: u32,
  ui_hz: u32,
  on_event: Channel<TelemetryEvent>,
  state: tauri::State<'_, Mutex<StreamState>>,
) -> Result<(), String> {
  // stop précédent si besoin
  stop_imu_stream(state.clone());

  let cancel = CancellationToken::new();
  state.lock().unwrap().cancel = Some(cancel.clone());

  tauri::async_runtime::spawn(async move {
    if let Err(e) = run_stream(port, baud, ui_hz, on_event, cancel).await {
      eprintln!("[imu_stream] error: {e}");
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

  let mut frames_total: u64 = 0;
  let mut imu_ok: u64 = 0;
  let mut imu_bad_len: u64 = 0;

  let mut chunk = ImuChunk {
    pc_us: vec![],
    t_us: vec![],
    ax: vec![],
    ay: vec![],
    az: vec![],
    gx: vec![],
    gy: vec![],
    gz: vec![],
  };

  let hz = ui_hz.max(1).min(240);
  let mut tick = tokio::time::interval(Duration::from_millis(1000 / hz as u64));
  let mut stat_tick = tokio::time::interval(Duration::from_millis(500));

  loop {
    tokio::select! {
      _ = cancel.cancelled() => break,

      _ = tick.tick() => {
        if !chunk.t_us.is_empty() {
          // envoie un chunk UI
          let send_chunk = std::mem::replace(&mut chunk, ImuChunk {
            pc_us: vec![],
            t_us: vec![], ax: vec![], ay: vec![], az: vec![], gx: vec![], gy: vec![], gz: vec![],
          });
          let _ = on_event.send(TelemetryEvent::ImuRawChunk(send_chunk));
        }
      }

      _ = stat_tick.tick() => {
        let _ = on_event.send(TelemetryEvent::Stats(Stats {
            frames_total,
            imu_ok,
            imu_bad_len,
            }));
        
        }
        
      read_res = serial.read(&mut buf) => {
        let n = match read_res {
          Ok(n) => n,
          Err(e) => return Err(e.to_string()),
        };
        if n == 0 { continue; }

        let frames = parser.feed(&buf[..n]);
        for f in frames {
          frames_total += 1;
          let pc_us: u64 = std::time::SystemTime::now()
  .duration_since(std::time::UNIX_EPOCH)
  .unwrap()
  .as_micros() as u64;

          handle_frame(&f, &mut chunk, &mut imu_ok, &mut imu_bad_len,  pc_us);
        }
      }
    }
  }

  Ok(())
}

fn handle_frame(f: &Frame, chunk: &mut ImuChunk, imu_ok: &mut u64, imu_bad_len: &mut u64, pc_us: u64) {
  if f.msg == BUS_MSG_PUBLISH && f.kind == BUS_KIND_STREAM && f.id == ID_IMU_RAW {
    match parse_imu_raw(&f.payload) {
      Some(s) => {
        *imu_ok += 1;
        chunk.pc_us.push(pc_us);
        chunk.t_us.push(s.t_us);
        chunk.ax.push(s.ax);
        chunk.ay.push(s.ay);
        chunk.az.push(s.az);
        chunk.gx.push(s.gx);
        chunk.gy.push(s.gy);
        chunk.gz.push(s.gz);
      }
      None => {
        *imu_bad_len += 1;
      }
    }
  }
}
