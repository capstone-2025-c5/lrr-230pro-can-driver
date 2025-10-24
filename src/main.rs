use anyhow::Result;
use socketcan::CANSocket;
use std::thread;
use std::time::Duration;

mod bitpack;
mod radar;
mod types;

fn main() -> Result<()> {
    // Create a CAN socket (requires `sudo ip link set can0 up type can bitrate 500000`)
    let can = CANSocket::open("can0")?;
    println!("ZLYLRR-230Pro Radar Driver started on can0...");

    // // Example: send vehicle state messages periodically
    // loop {
    //     radar::send_vehicle_state(&can, 0, 0, types::VehicleGear::Drive)?;
    //     radar::receive_frames(&can)?; // read any radar messages
    //     thread::sleep(Duration::from_millis(100));
    // }

    // Example loop: send required/recommended inputs; receive radar outputs.
    let mut t = 0f32;
    loop {
        // Required minimal inputs
        radar::send_abs_sts_run1(&can, 15.0, true)?;                  // 15 m/s
        radar::send_eps_sts_run(&can, 0.0, 0.0, true, false)?;        // angle/rate/cal
        radar::send_abs_sts_run2(&can, types::VehicleGear::Drive)?;   // gear

        // Recommended extra inputs
        radar::send_esc_sts_run1(&can, 0.1, 0.2, 0.01, true, true, true)?;
        radar::send_esc_sts_run2(&can, 0, 0, 0, 0)?;
        radar::send_abs_fault_info(&can, 50.0, 50.5, 49.8, 50.1)?; // km/h

        radar::receive_frames(&can)?;
        thread::sleep(Duration::from_millis(100));
        t += 0.1;
    }
}
