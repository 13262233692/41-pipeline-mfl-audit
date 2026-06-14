use std::fmt;

pub const MFL_MAGIC: [u8; 4] = [0x4D, 0x46, 0x4C, 0x31];

pub const FILE_HEADER_SIZE: usize = 64;
pub const FRAME_HEADER_SIZE: usize = 16;
pub const SAMPLE_BYTES: usize = 3;

#[allow(dead_code)]
#[repr(u8)]
pub enum AxisTag {
    Axial = 0x01,
    Transverse = 0x02,
    Radial = 0x03,
}

impl AxisTag {
    #[allow(dead_code)]
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x01 => Some(AxisTag::Axial),
            0x02 => Some(AxisTag::Transverse),
            0x03 => Some(AxisTag::Radial),
            _ => None,
        }
    }
}

impl fmt::Display for AxisTag {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            AxisTag::Axial => write!(f, "Axial"),
            AxisTag::Transverse => write!(f, "Transverse"),
            AxisTag::Radial => write!(f, "Radial"),
        }
    }
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct FileHeader {
    pub magic: [u8; 4],
    pub version: u16,
    pub num_channels: u16,
    pub num_axes: u8,
    pub sample_resolution_bits: u8,
    pub frame_rate_hz: u32,
    pub od_mm: f32,
    pub wall_thickness_mm: f32,
    pub sensor_spacing_deg: f32,
    pub timestamp_epoch: u64,
}

impl FileHeader {
    pub fn parse(buf: &[u8]) -> Option<Self> {
        if buf.len() < FILE_HEADER_SIZE {
            return None;
        }
        let magic: [u8; 4] = buf[0..4].try_into().ok()?;
        if magic != MFL_MAGIC {
            return None;
        }
        let version = u16::from_le_bytes([buf[4], buf[5]]);
        let num_channels = u16::from_le_bytes([buf[6], buf[7]]);
        let num_axes = buf[8];
        let sample_resolution_bits = buf[9];
        let frame_rate_hz = u32::from_le_bytes([buf[10], buf[11], buf[12], buf[13]]);
        let od_mm = f32::from_le_bytes([buf[14], buf[15], buf[16], buf[17]]);
        let wall_thickness_mm = f32::from_le_bytes([buf[18], buf[19], buf[20], buf[21]]);
        let sensor_spacing_deg = f32::from_le_bytes([buf[22], buf[23], buf[24], buf[25]]);
        let timestamp_epoch = u64::from_le_bytes([
            buf[26], buf[27], buf[28], buf[29], buf[30], buf[31], buf[32], buf[33],
        ]);
        Some(FileHeader {
            magic,
            version,
            num_channels,
            num_axes,
            sample_resolution_bits,
            frame_rate_hz,
            od_mm,
            wall_thickness_mm,
            sensor_spacing_deg,
            timestamp_epoch,
        })
    }

    pub fn sample_bytes(&self) -> usize {
        ((self.sample_resolution_bits as usize + 7) / 8).max(SAMPLE_BYTES)
    }

    pub fn frame_data_bytes(&self) -> usize {
        self.num_channels as usize * self.num_axes as usize * self.sample_bytes()
    }

    pub fn frame_total_bytes(&self) -> usize {
        FRAME_HEADER_SIZE + self.frame_data_bytes()
    }
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub struct FrameHeader {
    pub frame_index: u32,
    pub timestamp_us: u32,
    pub distance_mm: f32,
    pub velocity_mms: f32,
    pub flags: u16,
}

impl FrameHeader {
    pub fn parse(buf: &[u8]) -> Option<Self> {
        if buf.len() < FRAME_HEADER_SIZE {
            return None;
        }
        let frame_index = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        let timestamp_us = u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]);
        let distance_mm = f32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        let velocity_mms = f32::from_le_bytes([buf[12], buf[13], buf[14], buf[15]]);
        let flags = 0u16;
        Some(FrameHeader {
            frame_index,
            timestamp_us,
            distance_mm,
            velocity_mms,
            flags,
        })
    }
}

#[inline]
pub fn decode_signed_sample(raw: &[u8], bits: u8) -> i32 {
    let nbytes = ((bits as usize + 7) / 8).min(raw.len());
    let mut val: u32 = 0;
    for i in 0..nbytes {
        val |= (raw[i] as u32) << (i * 8);
    }
    let bit_count = bits as u32;
    if bit_count < 32 && (val & (1u32 << (bit_count - 1))) != 0 {
        val |= !0u32 << bit_count;
    }
    val as i32
}
