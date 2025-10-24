use core::fmt;

pub const ID_ESC_RUN1: u32 = 0x141;
pub const ID_ESC_RUN2: u32 = 0x142;
pub const ID_EPS_RUN: u32 = 0x221;
pub const ID_ABS_FAULT: u32 = 0x310;
pub const ID_ABS_RUN1: u32 = 0x330;
pub const ID_ABS_RUN2: u32 = 0x210;

pub const ID_FRS_STATUS: u32 = 0x80;
pub const ID_FRS_OBJ_PART1_BASE: u32 = 0x60; // 0x60..=0x6F (Part1)
pub const ID_FRS_OBJ_PART2_BASE: u32 = 0x70; // 0x70..=0x7F (Part2)

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InvalidStatusValueError(pub u8);

impl fmt::Display for InvalidStatusValueError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Invalid Status value: 0x{:02X}", self.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CalibrationStatus {
    /// 0x00: Unknown (also marked as an invalid value)
    Unknown = 0x00,
    /// 0x01: Calibrated (Valid state)
    Calibrated = 0x01,
    /// 0x02: Sensor mis-alignment detected (Valid state)
    SensorMisalignmentDetected = 0x02,
    /// 0x03: Calibration in progress (also marked as an invalid value)
    CalibrationInProgress = 0x03,
    /// 0x04: Uncalibratable (also marked as an invalid value)
    Uncalibratable = 0x04,
}

impl TryFrom<u8> for CalibrationStatus {
    type Error = InvalidStatusValueError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(Self::Unknown),
            0x01 => Ok(Self::Calibrated),
            0x02 => Ok(Self::SensorMisalignmentDetected),
            0x03 => Ok(Self::CalibrationInProgress),
            0x04 => Ok(Self::Uncalibratable),
            // Values 0x05-0x07 are reserved and invalid, so we return an error.
            // All values above 0x07 are also undefined and invalid.
            _ => Err(InvalidStatusValueError(value)),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum VehicleGear {
    /// 0x00: Unknown (also marked as an invalid value)
    Unknown = 0x00,
    /// 0x01: Reverse
    Reverse = 0x01,
    /// 0x02: Drive
    Drive = 0x02,
    /// 0x03: Neutral
    Neutral = 0x03,
}

impl TryFrom<u8> for VehicleGear {
    type Error = InvalidStatusValueError;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x00 => Ok(Self::Unknown),
            0x01 => Ok(Self::Reverse),
            0x02 => Ok(Self::Drive),
            0x03 => Ok(Self::Neutral),
            // Values 0x04-0xFF are reserved and invalid, so we return an error.
            _ => Err(InvalidStatusValueError(value)),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MotionPattern {
    Unknown, StationaryInvalid, Stopped, Moving, CrossingInvalid, Reserved(u8),
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObjectType {
    Unknown, FourWheeler, TwoWheelerInvalid, PedestrianInvalid,
}

#[derive(Debug, Clone)]
pub struct FrsStatus {
    pub latency_ms: f32,
    pub timestamp_s: u16,
    pub host_speed_mps: f32,
    pub blocked: bool,
    pub fail_flag: bool,
    pub meas_enabled: bool,
    pub host_yaw_dps: f32,
    pub alive_counter: u8,
    pub misalign_status: CalibrationStatus,
    pub hw_error: bool,
}

#[derive(Debug, Clone)]
pub struct ObjPart1 {
    pub obj_id: u8,                 // 0xFF invalid
    pub x_pos_stdev_m: f32,         // 0..12.7 m
    pub update_flag: bool,
    pub y_pos_stdev_m: f32,         // 0..12.7 m
    pub valid_flag: bool,
    pub obstacle_prob_pct: f32,     // not recommended; prefer exst_prob
    pub motion_pattern: MotionPattern,
    pub x_acc_rel_mps2: f32,        // -9.6 .. 9.45
    pub x_vel_rel_stdev_mps: f32,   // 0..6.35
    pub alive_counter: u8,          // 0..15
    pub exst_prob_pct: f32,         // recommended presence prob
    pub checksum: u8,
}

#[derive(Debug, Clone)]
pub struct ObjPart2 {
    pub x_vel_rel_mps: f32,         // -102.4..102.3
    pub y_pos_m: f32,               // -64..63.984375
    pub obj_type: ObjectType,
    pub x_pos_m: f32,               // 0..255.984375
    pub alive_counter: u8,          // 0..15
    pub meas_flag_extrapolated: bool,
    pub y_vel_rel_mps: f32,         // -102.4..102.3
    pub checksum: u8,
}