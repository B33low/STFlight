// src-tauri/src/lib.rs
mod imu_stream;
mod protocol;

use std::sync::Mutex;

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
  tauri::Builder::default()
    .manage(Mutex::new(imu_stream::StreamState::default()))
    .invoke_handler(tauri::generate_handler![
      imu_stream::list_ports,
      imu_stream::start_imu_stream,
      imu_stream::stop_imu_stream,
    ])
    .run(tauri::generate_context!())
    .expect("error while running tauri application");
}
