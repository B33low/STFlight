use std::io::Cursor;

use byteorder::{LittleEndian, ReadBytesExt};

use crate::{
    protocol::{Frame, MsgKind, MsgType},
    telemetry::{ImuChunk, Stats, TelemetryEvent},
};

use super::Processor;

const BUS_MSG_PUBLISH: u8 = 1;
const BUS_KIND_STREAM: u8 = 3;
const ID_IMU_RAW: u8 = 1;

// <hhhhhhhI => 7*i16 + u32 = 18 bytes
const IMU_SIZE: usize = 18;

#[derive(Debug, Clone, Copy)]
struct ImuRaw {
    ax: i16, ay: i16, az: i16,
    gx: i16, gy: i16, gz: i16,
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

#[derive(Default)]
pub struct ImuProcessor {
    ok: u64,
    bad_len: u64,
    chunk: ImuChunk,
}

impl ImuProcessor {
    pub fn new() -> Self {
        Self {
            ok: 0,
            bad_len: 0,
            chunk: ImuChunk {
                pc_us: vec![],
                t_us: vec![],
                ax: vec![], ay: vec![], az: vec![],
                gx: vec![], gy: vec![], gz: vec![],
            },
        }
    }
}

impl Processor for ImuProcessor {
    fn on_frame(&mut self, f: &Frame, pc_us: u64) {
        if f.msg == MsgType::BusMsgPublish && f.kind == MsgKind::BusKindStream && f.id == ID_IMU_RAW {
            match parse_imu_raw(&f.payload) {
                Some(s) => {
                    self.ok += 1;
                    self.chunk.pc_us.push(pc_us);
                    self.chunk.t_us.push(s.t_us);
                    self.chunk.ax.push(s.ax);
                    self.chunk.ay.push(s.ay);
                    self.chunk.az.push(s.az);
                    self.chunk.gx.push(s.gx);
                    self.chunk.gy.push(s.gy);
                    self.chunk.gz.push(s.gz);
                }
                None => self.bad_len += 1,
            }
        }
    }

    fn on_tick(&mut self) -> Option<TelemetryEvent> {
        if self.chunk.t_us.is_empty() {
            return None;
        }
        let send = std::mem::replace(&mut self.chunk, ImuChunk {
            pc_us: vec![],
            t_us: vec![],
            ax: vec![], ay: vec![], az: vec![],
            gx: vec![], gy: vec![], gz: vec![],
        });
        Some(TelemetryEvent::ImuRawChunk(send))
    }

    fn add_stats(&self, out: &mut Stats) {
        out.imu_ok = self.ok;
        out.imu_bad_len = self.bad_len;
    }
}
