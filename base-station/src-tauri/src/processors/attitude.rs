use std::io::Cursor;

use byteorder::{LittleEndian, ReadBytesExt};

use crate::{
    protocol::{Frame,MsgType,MsgKind},
    telemetry::{AttitudeChunk, TelemetryEvent, Stats},
};

use super::Processor;

const BUS_MSG_PUBLISH: u8 = 1;
const BUS_KIND_STREAM: u8 = 3;
const BUS_KIND_STATE: u8 = 1;
const ID_ATT_STATE: u8 = 3;

// <fff => 3*f32 = 12 bytes
const ATT_SIZE: usize = 12;

#[derive(Debug, Clone, Copy)]
struct AttEuler {
    pitch: f32,
    roll: f32,
    yaw: f32,
}

fn parse_att_euler(payload: &[u8]) -> Option<AttEuler> {
    if payload.len() != ATT_SIZE {
        return None;
    }
    let mut rdr = Cursor::new(payload);
    Some(AttEuler {
    roll:  rdr.read_f32::<LittleEndian>().ok()?,
    pitch: rdr.read_f32::<LittleEndian>().ok()?,
    yaw:   rdr.read_f32::<LittleEndian>().ok()?,
})

}

#[derive(Default)]
pub struct AttitudeProcessor {
    ok: u64,
    bad_len: u64,
    chunk: AttitudeChunk,
    unit: &'static str,
}

impl AttitudeProcessor {
    pub fn new(unit: &'static str) -> Self {
        Self {
            ok: 0,
            bad_len: 0,
            unit,
            chunk: AttitudeChunk {
                pc_us: vec![],
                roll: vec![],
                pitch: vec![],
                yaw: vec![],
                unit,
            },
        }
    }
}

impl Processor for AttitudeProcessor {
    fn on_frame(&mut self, f: &Frame, pc_us: u64) {
        if f.msg == MsgType::BusMsgPublish && f.kind == MsgKind::BusKindState && f.id == ID_ATT_STATE {
            match parse_att_euler(&f.payload) {
                Some(a) => {
                    self.ok += 1;
                    self.chunk.pc_us.push(pc_us);
                    self.chunk.roll.push(a.roll);
                    self.chunk.pitch.push(a.pitch);
                    self.chunk.yaw.push(a.yaw);
                }
                None => self.bad_len += 1,
            }
        }
    }

    fn on_tick(&mut self) -> Option<TelemetryEvent> {
        if self.chunk.pc_us.is_empty() {
            return None;
        }
        let send = std::mem::replace(&mut self.chunk, AttitudeChunk {
            pc_us: vec![],
            roll: vec![],
            pitch: vec![],
            yaw: vec![],
            unit: self.unit,
        });
        Some(TelemetryEvent::AttitudeChunk(send))
    }

    fn add_stats(&self, out: &mut Stats) {
        out.att_ok = self.ok;
        out.att_bad_len = self.bad_len;
    }
}
