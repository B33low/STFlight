use serde::Serialize;

#[derive(Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct PortInfo {
    pub name: String,
}

#[derive(Clone, Serialize, Default)]
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

#[derive(Clone, Serialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct GyroSetpointChunk {
    pub pc_us: Vec<u64>,
    pub t_us: Vec<u32>,
    pub gx: Vec<i16>,
    pub gy: Vec<i16>,
    pub gz: Vec<i16>,
}

#[derive(Clone, Serialize, Default)]
#[serde(rename_all = "camelCase")]
pub struct AttitudeChunk {
    pub pc_us: Vec<u64>,
    pub roll: Vec<f32>,
    pub pitch: Vec<f32>,
    pub yaw: Vec<f32>,
    /// Optional, if you want to explicitly tag units
    pub unit: &'static str, // "rad" or "deg"
}

#[derive(Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct Stats {
    pub frames_total: u64,

    pub imu_ok: u64,
    pub imu_bad_len: u64,

    pub att_ok: u64,
    pub att_bad_len: u64,

    pub gyro_setpoint_ok: u64,
    pub gyro_setpoint_bad_len: u64,
}

#[derive(Clone, Serialize)]
#[serde(tag = "event", content = "data", rename_all = "camelCase")]
pub enum TelemetryEvent {
    ImuRawChunk(ImuChunk),
    AttitudeChunk(AttitudeChunk),
    GyroSetpointChunk(GyroSetpointChunk),
    Stats(Stats),
}
