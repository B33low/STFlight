mod protocol;
mod telemetry;
mod serial_manager;
mod processors;

use std::sync::Mutex;

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    tauri::Builder::default()
        .manage(Mutex::new(serial_manager::StreamState::default()))
        .invoke_handler(tauri::generate_handler![
            serial_manager::list_ports,
            serial_manager::start_telemetry_stream,
            serial_manager::stop_telemetry_stream,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}
