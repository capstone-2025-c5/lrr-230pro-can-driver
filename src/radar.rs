use anyhow::{anyhow, Result};
use socketcan::{CanFrame, CanSocket, Socket, StandardId, EmbeddedFrame};

use crate::bitpack::*;
use crate::types::*;

pub fn send_vehicle_state(
    can: &CanSocket,
    veh_speed_mps: f32,
    _yaw_rate_rad_s: f32,
    gear: VehicleGear,
) -> Result<()> {
    // Required minimal set delegates to full senders with neutral defaults.
    send_abs_sts_run1(can, veh_speed_mps, true)?;           // 0x330
    send_eps_sts_run(can, 0.0, 0.0, true, false)?;          // 0x221
    send_abs_sts_run2(can, gear)?;                          // 0x210
    Ok(())
}

// 0x141 esc_sts_run_1: lateral/longitudinal acc, yaw rate + valid bits
pub fn send_esc_sts_run1(
    can: &CanSocket,
    lat_acc_mps2: f32,
    lon_acc_mps2: f32,
    yaw_rate_rad_s: f32,
    yaw_valid: bool,
    lon_valid: bool,
    lat_valid: bool,
) -> Result<()> {
    // Factor/offset per spec:
    // Lateral/Longitudinal ACC: factor 0.027127, min -21.593..21.593 (16-bit signed)
    // Yaw rate: factor 0.00021326, signed 16-bit
    let mut d = [0u8; 8];

    // ESC_LateralACC @ startbit 8, len 16 (Motorola, signed)
    let raw_lat = (lat_acc_mps2 / 0.027127).round() as i16;
    set_bits_m(&mut d, 8, 16, raw_lat as u16 as u64);

    // ESC_LongitudinalACC @ startbit 24, len 16
    let raw_lon = (lon_acc_mps2 / 0.027127).round() as i16;
    set_bits_m(&mut d, 24, 16, raw_lon as u16 as u64);

    // ESC_VehDynYawRate @ startbit 40, len 16
    let raw_yaw = (yaw_rate_rad_s / 0.000_213_26).round() as i16;
    set_bits_m(&mut d, 40, 16, raw_yaw as u16 as u64);

    // Validity bits @ 48..50
    set_bits_m(&mut d, 48, 1, yaw_valid as u64);
    set_bits_m(&mut d, 49, 1, lon_valid as u64);
    set_bits_m(&mut d, 50, 1, lat_valid as u64);

    let sid = StandardId::new((ID_ESC_RUN1 & 0x7FF) as u16).ok_or(anyhow!("invalid std id"))?;
    let frame = CanFrame::new(sid, &d).ok_or(anyhow!("failed to create frame"))?;
    can.write_frame(&frame)?;
    Ok(())
}

// 0x142 esc_sts_run_2: wheel-speed validities FL/FR/RL/RR (2 bits each)
pub fn send_esc_sts_run2(
    can: &CanSocket,
    fl_valid: u8,
    fr_valid: u8,
    rl_valid: u8,
    rr_valid: u8,
) -> Result<()> {
    let mut d = [0u8; 8];
    set_bits_m(&mut d, 0, 2, (fl_valid & 0x3) as u64);
    set_bits_m(&mut d, 2, 2, (fr_valid & 0x3) as u64);
    set_bits_m(&mut d, 4, 2, (rl_valid & 0x3) as u64);
    set_bits_m(&mut d, 6, 2, (rr_valid & 0x3) as u64);
    let sid = StandardId::new((ID_ESC_RUN2 & 0x7FF) as u16).ok_or(anyhow!("invalid std id"))?;
    let frame = CanFrame::new(sid, &d).ok_or(anyhow!("failed to create frame"))?;
    can.write_frame(&frame)?;
    Ok(())
}

// 0x221 eps_sts_run: steering angle/velocity + status bits
pub fn send_eps_sts_run(
    can: &CanSocket,
    angle_deg: f32,
    angle_spd_dps: f32,
    cal_ok: bool,
    fault: bool,
) -> Result<()> {
    let mut d = [0u8; 8];
    // eps_pm_angle @ 8 len16 factor 0.1
    let raw_angle = (angle_deg / 0.1).round().clamp(-780.0, 779.9) as i16;
    set_bits_m(&mut d, 8, 16, raw_angle as u16 as u64);
    // eps_pm_angle_spd @ 16 len8 factor 4
    let raw_spd = (angle_spd_dps / 4.0).round().clamp(0.0, 255.0) as u8;
    set_bits_m(&mut d, 16, 8, raw_spd as u64);
    // eps_sts_cal @30, eps_sts_ft @31
    set_bits_m(&mut d, 30, 1, cal_ok as u64);
    set_bits_m(&mut d, 31, 1, fault as u64);

    let sid = StandardId::new((ID_EPS_RUN & 0x7FF) as u16).ok_or(anyhow!("invalid std id"))?;
    let frame = CanFrame::new(sid, &d).ok_or(anyhow!("failed to create frame"))?;
    can.write_frame(&frame)?;
    Ok(())
}

// 0x310 abs_fault_info: four wheel speeds (km/h, factor 0.05625, 13-bit each)
pub fn send_abs_fault_info(
    can: &CanSocket,
    fl_kmph: f32,
    fr_kmph: f32,
    rl_kmph: f32,
    rr_kmph: f32,
) -> Result<()> {
    let mut d = [0u8; 8];
    let enc = |v: f32| -> u16 { (v / 0.05625).round() as u16 & 0x1FFF };

    set_bits_m(&mut d, 11, 13, enc(fl_kmph) as u64);
    set_bits_m(&mut d, 27, 13, enc(fr_kmph) as u64);
    set_bits_m(&mut d, 43, 13, enc(rl_kmph) as u64);
    set_bits_m(&mut d, 56, 13, enc(rr_kmph) as u64);

    let sid = StandardId::new((ID_ABS_FAULT & 0x7FF) as u16).ok_or(anyhow!("invalid std id"))?;
    let frame = CanFrame::new(sid, &d).ok_or(anyhow!("failed to create frame"))?;
    can.write_frame(&frame)?;
    Ok(())
}

// 0x330 abs_sts_run1: vehicle speed + valid
pub fn send_abs_sts_run1(can: &CanSocket, veh_speed_mps: f32, valid: bool) -> Result<()> {
    // Spec is km/h with factor 0.05625 and 13-bit at startbit 8.
    let kmph = veh_speed_mps * 3.6;
    let mut d = [0u8; 8];
    let raw = (kmph / 0.05625).round() as u16 & 0x1FFF;
    set_bits_m(&mut d, 8, 13, raw as u64);
    // abs_sts_veh_spd_valid at startbit 16 (1 bit): 0=Valid, 1=Invalid
    // Our boolean 'valid = true' => put 0; false => 1
    set_bits_m(&mut d, 16, 1, (!valid) as u64);
    let sid = StandardId::new((ID_ABS_RUN1 & 0x7FF) as u16).ok_or(anyhow!("invalid std id"))?;
    let frame = CanFrame::new(sid, &d).ok_or(anyhow!("failed to create frame"))?;
    can.write_frame(&frame)?;
    Ok(())
}

// 0x210 abs_sts_run2: gear
pub fn send_abs_sts_run2(can: &CanSocket, gear: VehicleGear) -> Result<()> {
    let mut d = [0u8; 8];
    // vcu_sts_gear @ startbit 2, len 2:
    set_bits_m(&mut d, 2, 2, (gear as u8 & 0x3) as u64);
    let sid = StandardId::new((ID_ABS_RUN2 & 0x7FF) as u16).ok_or(anyhow!("invalid std id"))?;
    let frame = CanFrame::new(sid, &d).ok_or(anyhow!("failed to create frame"))?;
    can.write_frame(&frame)?;
    Ok(())
}

// Receiving/Parsing Frames
pub fn receive_frames(can: &CanSocket) -> Result<()> {
    // Non-blocking single read; wrap in your loop.
    if let Ok(frame) = can.read_frame() {
        let data = frame.data();
        // convert the frame id to a raw u32 for comparison with our constants
        let id_raw: u32 = match frame.id() {
            socketcan::Id::Standard(sid) => sid.as_raw() as u32,
            socketcan::Id::Extended(eid) => eid.as_raw(),
        };

        match id_raw {
            x if x == ID_FRS_STATUS => {
                let st = parse_frs_status(data)?;
                println!("FRS Status: {:?}", st);
            }
            x if (ID_FRS_OBJ_PART1_BASE..=ID_FRS_OBJ_PART1_BASE + 0x0F).contains(&x) => {
                let idx = x - ID_FRS_OBJ_PART1_BASE;
                let p1 = parse_object_part1(data)?;
                println!("Obj{:02} P1: {:?}", idx, p1);
            }
            x if (ID_FRS_OBJ_PART2_BASE..=ID_FRS_OBJ_PART2_BASE + 0x0F).contains(&x) => {
                let idx = x - ID_FRS_OBJ_PART2_BASE;
                let p2 = parse_object_part2(data)?;
                println!("Obj{:02} P2: {:?}", idx, p2);
            }
            _ => {}
        }
    }
    Ok(())
}

pub fn parse_frs_status(d: &[u8]) -> Result<FrsStatus> {
    if d.len() < 8 { return Err(anyhow!("FRS_Status len")); }

    let latency = get_bits_m(d, 2, 6) as f32 * 2.0; // ms
    let ts = get_bits_m(d, 18, 16) as u16;          // s
    let host_speed = (get_bits_m(d, 38, 12) as i64 - (20.0 / 0.025) as i64) as f32 * 0.025; // factor 0.025 offset -20
    let blk = get_bits_m(d, 40, 1) != 0;
    let fail = get_bits_m(d, 41, 1) != 0;
    let meas_en = get_bits_m(d, 42, 1) != 0;
    let host_yaw = (sign_extend(get_bits_m(d, 43, 11), 11) as f32) * 0.1; // deg/s
    let alive = get_bits_m(d, 48, 4) as u8;
    let mis = get_bits_m(d, 52, 3) as u8;
    let hw_err = get_bits_m(d, 55, 1) != 0;

    let misalign = CalibrationStatus::try_from(mis).unwrap_or(CalibrationStatus::Unknown);

    Ok(FrsStatus {
        latency_ms: latency,
        timestamp_s: ts,
        host_speed_mps: host_speed,
        blocked: blk,
        fail_flag: fail,
        meas_enabled: meas_en,
        host_yaw_dps: host_yaw,
        alive_counter: alive,
        misalign_status: misalign,
        hw_error: hw_err,
    })
}

pub fn parse_object_part1(d: &[u8]) -> Result<ObjPart1> {
    if d.len() < 8 { return Err(anyhow!("P1 len")); }

    let obj_id = get_bits_m(d, 0, 8) as u8;
    let x_pos_stdev = get_bits_m(d, 8, 7) as f32 * 0.1;
    let update_flag = get_bits_m(d, 15, 1) != 0;
    let y_pos_stdev = get_bits_m(d, 16, 7) as f32 * 0.1;
    let valid_flag = get_bits_m(d, 23, 1) != 0;

    let obstacle_prob = get_bits_m(d, 24, 5) as f32 * 3.2258; // % (not recommended)
    let motion_raw = get_bits_m(d, 29, 3) as u8;
    let motion = match motion_raw {
        0x00 => MotionPattern::Unknown,
        0x01 => MotionPattern::StationaryInvalid,
        0x02 => MotionPattern::Stopped,
        0x03 => MotionPattern::Moving,
        0x04 => MotionPattern::CrossingInvalid,
        x    => MotionPattern::Reserved(x),
    };

    let x_acc_rel = (sign_extend(get_bits_m(d, 33, 7), 7) as f32) * 0.15 - 9.6; // per table
    let x_vel_rel_stdev = get_bits_m(d, 42, 7) as f32 * 0.05;

    let alive = get_bits_m(d, 48, 4) as u8;
    let exst_prob = get_bits_m(d, 52, 6) as f32 * 1.5873; // %
    let checksum = get_bits_m(d, 56, 8) as u8;

    Ok(ObjPart1 {
        obj_id,
        x_pos_stdev_m: x_pos_stdev,
        update_flag,
        y_pos_stdev_m: y_pos_stdev,
        valid_flag,
        obstacle_prob_pct: obstacle_prob,
        motion_pattern: motion,
        x_acc_rel_mps2: x_acc_rel,
        x_vel_rel_stdev_mps: x_vel_rel_stdev,
        alive_counter: alive,
        exst_prob_pct: exst_prob,
        checksum,
    })
}

pub fn parse_object_part2(d: &[u8]) -> Result<ObjPart2> {
    if d.len() < 8 { return Err(anyhow!("P2 len")); }

    let x_vel_rel = (sign_extend(get_bits_m(d, 13, 11), 11) as f32) * 0.1;
    let y_pos = (sign_extend(get_bits_m(d, 16, 13), 13) as f32) * 0.015625;

    let obj_type = match get_bits_m(d, 32, 2) as u8 {
        0x00 => ObjectType::Unknown,
        0x01 => ObjectType::FourWheeler,
        0x02 => ObjectType::TwoWheelerInvalid,
        0x03 => ObjectType::PedestrianInvalid,
        _ => ObjectType::Unknown,
    };

    let x_pos = (get_bits_m(d, 34, 14) as f32) * 0.015625; // 0..255.984375
    let alive = get_bits_m(d, 48, 4) as u8;
    let meas_flag_extrapolated = get_bits_m(d, 52, 1) != 0;
    let y_vel_rel = (sign_extend(get_bits_m(d, 53, 11), 11) as f32) * 0.1;
    let checksum = get_bits_m(d, 56, 8) as u8;

    Ok(ObjPart2 {
        x_vel_rel_mps: x_vel_rel,
        y_pos_m: y_pos,
        obj_type,
        x_pos_m: x_pos,
        alive_counter: alive,
        meas_flag_extrapolated,
        y_vel_rel_mps: y_vel_rel,
        checksum,
    })
}
