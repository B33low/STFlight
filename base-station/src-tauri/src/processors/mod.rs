use crate::{protocol::Frame, telemetry::{Stats, TelemetryEvent}};

pub trait Processor: Send {
    fn on_frame(&mut self, f: &Frame, pc_us: u64);
    fn on_tick(&mut self) -> Option<TelemetryEvent>;
    fn add_stats(&self, out: &mut Stats);
}

pub mod imu;
pub mod attitude;
