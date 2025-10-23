use socketcan::{CANSocket, CANFrame};
use anyhow::Result;
use std::thread;
use std::time::Duration;
mod radar;
mod types;

fn main() -> Result<()> {
    // Create a CAN socket (requires `sudo ip link set can0 up type can bitrate 500000`)
    let can = CANSocket::open("can0")?;
    println!("ZLYLRR-230Pro Radar Driver started on can0...");

    // Example: send vehicle state messages periodically
    loop {
        radar::send_vehicle_state(&can, 0, 0, types::VehicleGear::Drive)?;
        radar::receive_frames(&can)?; // read any radar messages
        thread::sleep(Duration::from_millis(100));
    }
}
