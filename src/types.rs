pub const ID_ESC_RUN1: u32 = 0x141;
pub const ID_ESC_RUN2: u32 = 0x142;
pub const ID_EPS_RUN: u32 = 0x221;
pub const ID_ABS_RUN2: u32 = 0x210;
pub const ID_ABS_RUN1: u32 = 0x330;

pub const ID_FRS_STATUS: u32 = 0x80;
pub const ID_FRS_OBJ_PART1_BASE: u32 = 0x60; // adjust according to spec
pub const ID_FRS_OBJ_PART2_BASE: u32 = 0x70;

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
            0x00 => Ok(CalibrationStatus::Unknown),
            0x01 => Ok(CalibrationStatus::Calibrated),
            0x02 => Ok(CalibrationStatus::SensorMisalignmentDetected),
            0x03 => Ok(CalibrationStatus::CalibrationInProgress),
            0x04 => Ok(CalibrationStatus::Uncalibratable),
            // Values 0x05-0x07 are reserved and invalid, so we return an error.
            // All values above 0x07 are also undefined and invalid.
            0x05..=0xFF => Err(InvalidStatusValueError(value)),
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
            0x00 => Ok(VehicleGear::Unknown),
            0x01 => Ok(VehicleGear::Reverse),
            0x02 => Ok(VehicleGear::Drive),
            0x03 => Ok(VehicleGear::Neutral),
            // Values 0x04-0xFF are reserved and invalid, so we return an error.
            0x04..=0xFF => Err(InvalidStatusValueError(value)),
        }
    }
}
