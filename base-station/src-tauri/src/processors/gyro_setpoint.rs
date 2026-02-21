use std::io::Cursor;

use byteorder::{LittleEndian, ReadBytesExt};

use crate::{
    protocol::{Frame, MsgKind, MsgType},
    telemetry::{GyroSetpointChunk, Stats, TelemetryEvent},
};

use super::Processor;

const BUS_MSG_PUBLISH: u8 = 1;
const BUS_KIND_STREAM: u8 = 3;
const BUS_KIND_STATE: u8 = 1;
const ID_GYRO_SETPOINT_STATE: u8 = 5;

// <hhh => 3*i16 = 6 bytes
const GYRO_SETPOINT_SIZE: usize = 6;

#[derive(Debug, Clone, Copy)]
struct GyroSetpoint {
    rx: i16,
    ry: i16,
    rz: i16,
}

fn parse_gyro_setpoint(payload: &[u8]) -> Option<GyroSetpoint> {
    if payload.len() != GYRO_SETPOINT_SIZE {
        return None;
    }
    let mut rdr = Cursor::new(payload);
    Some(GyroSetpoint {
        rx: rdr.read_i16::<LittleEndian>().ok()?,
        ry: rdr.read_i16::<LittleEndian>().ok()?,
        rz: rdr.read_i16::<LittleEndian>().ok()?,
    })
}

#[derive(Default)]
pub struct GyroSetpointProcessor {
    ok: u64,
    bad_len: u64,
    chunk: GyroSetpointChunk,
}

impl GyroSetpointProcessor {
    pub fn new() -> Self {
        Self {
            ok: 0,
            bad_len: 0,
            chunk: GyroSetpointChunk {
                pc_us: vec![],
                t_us: vec![],
                rx: vec![],
                ry: vec![],
                rz: vec![],
            },
        }
    }
}

impl Processor for GyroSetpointProcessor {
    fn on_frame(&mut self, f: &Frame, pc_us: u64) {
        if f.msg == MsgType::BusMsgPublish
            && f.kind == MsgKind::BusKindState
            && f.id == ID_GYRO_SETPOINT_STATE
        {
            match parse_gyro_setpoint(&f.payload) {
                Some(a) => {
                    self.ok += 1;
                    self.chunk.pc_us.push(pc_us);
                    self.chunk.rx.push(a.rx);
                    self.chunk.ry.push(a.ry);
                    self.chunk.rz.push(a.rz);
                }
                None => self.bad_len += 1,
            }
        }
    }

    fn on_tick(&mut self) -> Option<TelemetryEvent> {
        if self.chunk.pc_us.is_empty() {
            return None;
        }
        let send = std::mem::replace(
            &mut self.chunk,
            GyroSetpointChunk {
                pc_us: vec![],
                t_us: vec![],
                rx: vec![],
                ry: vec![],
                rz: vec![],
            },
        );
        Some(TelemetryEvent::GyroSetpointChunk(send))
    }

    fn add_stats(&self, out: &mut Stats) {
        out.gyro_setpoint_ok = self.ok;
        out.gyro_setpoint_bad_len = self.bad_len;
    }
}
