use socketcan::{CANSocket, CANFrame};
use anyhow::Result;
use bitfield::bitfield;

pub fn send_vehicle_state(can: &CANSocket, speed_mps: f32, yaw_rate_rad: f32, gear: VehicleGear) -> Result<()> {
    // ABS vehicle speed (0x330)
    let abs_speed = (speed_mps / 0.05625) as u16;
    let mut data = [0u8; 8];
    data[1] = (abs_speed & 0xFF) as u8;
    data[2] = (abs_speed >> 8) as u8;
    let frame = CANFrame::new(ID_ABS_RUN1, &data, false, false)?;
    can.write_frame(&frame)?;

    // EPS yaw rate (0x221)
    let yaw = (yaw_rate_rad / 0.00021326) as u16;
    let mut eps = [0u8; 8];
    eps[1] = (yaw & 0xFF) as u8;
    eps[2] = (yaw >> 8) as u8;
    let frame = CANFrame::new(ID_EPS_RUN, &eps, false, false)?;
    can.write_frame(&frame)?;

    // Gear signal (0x210)
    let mut gear_data = [0u8; 8];
    gear_data[0] = gear & 0x03; // 0x1=R, 0x2=D, 0x3=N
    let frame = CANFrame::new(ID_ABS_RUN2, &gear_data, false, false)?;
    can.write_frame(&frame)?;

    Ok(())
}

pub fn receive_frames(can: &CANSocket) -> Result<()> {
    if let Ok(frame) = can.read_frame() {
        match frame.id() {
            types::ID_FRS_STATUS => parse_frs_status(&frame.data())?,
            id if (0x60..=0x6F).contains(&id) => parse_object_part1(id, &frame.data())?,
            id if (0x70..=0x7F).contains(&id) => parse_object_part2(id, &frame.data())?,
            _ => {}
        }
    }
    Ok(())
}

fn parse_frs_status(data: &[u8]) -> Result<()> {
    let latency = ((data[0] >> 2) & 0x3F) as f32 * 2.0;
    let timestamp = ((data[2] as u16) << 8 | data[1] as u16) as f32;
    let yaw = (((data[5] as u16) << 3 | (data[4] >> 5) as u16) as f32 * 0.1) - 102.4;
    println!("Radar Status: latency={latency}ms ts={timestamp}s yaw={yaw}°/s");
    Ok(())
}

fn parse_object_part1(id: u32, data: &[u8]) -> Result<()> {
    let obj_id = data[0];
    let x_pos_stdev = ((data[1] & 0x7F) as f32) * 0.1;
    let valid_flag = (data[2] >> 7) & 0x1;
    println!("Obj{:02}: x_stdev={x_pos_stdev} valid={valid_flag}", id - types::ID_FRS_OBJ_PART1_BASE);
    Ok(())
}

fn parse_object_part2(id: u32, data: &[u8]) -> Result<()> {
    let y_vel_bits = ((data[6] as u16) << 8 | data[5] as u16) >> 5;
    let y_vel = (y_vel_bits as f32) * 0.1 - 102.4;
    println!("Obj{:02}: y_vel={y_vel} m/s", id - types::ID_FRS_OBJ_PART2_BASE);
    Ok(())
}
