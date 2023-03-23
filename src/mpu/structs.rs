use fixed::{types, FixedI32};

/// A quaternion is a mathematical way of representing angles.
/// These are not very intuitive, but this is what the hardware returns.
/// You should convert these to `YawPitchRoll` before doing further logic on them.
///
/// Warning: This struct uses a `FixedI32` with 30 fractional bits. You may want to convert these to a more useful format.
#[derive(Copy, Clone, Debug)]
#[allow(missing_docs)]
pub struct Quaternion {
    pub w: FixedI32<types::extra::U30>,
    pub x: FixedI32<types::extra::U30>,
    pub y: FixedI32<types::extra::U30>,
    pub z: FixedI32<types::extra::U30>,
}

impl Quaternion {
    pub(crate) fn from_bytes(bytes: &[u8]) -> Self {
        assert_eq!(bytes.len(), 16);

        let w =
            FixedI32::<types::extra::U30>::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        let x =
            FixedI32::<types::extra::U30>::from_be_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]);
        let y = FixedI32::<types::extra::U30>::from_be_bytes([
            bytes[8], bytes[9], bytes[10], bytes[11],
        ]);
        let z = FixedI32::<types::extra::U30>::from_be_bytes([
            bytes[12], bytes[13], bytes[14], bytes[15],
        ]);
        Quaternion { w, x, y, z }
    }
}

/// The accelerometer values.
/// They are in the range of [-2G, 2G].
#[derive(Copy, Clone, Debug)]
#[allow(missing_docs)]
pub struct Accel {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

impl Accel {
    pub(crate) fn from_bytes(data: [u8; 6]) -> Self {
        let x = [data[0], data[1]];
        let y = [data[2], data[3]];
        let z = [data[4], data[5]];
        Self {
            x: i16::from_be_bytes(x),
            y: i16::from_be_bytes(y),
            z: i16::from_be_bytes(z),
        }
    }
}

/// The gyroscope values.
/// They are in the range of [-2000 deg/second, 2000 deg/second].
#[derive(Copy, Clone, Debug)]
#[allow(missing_docs)]
pub struct Gyro {
    pub x: i16,
    pub y: i16,
    pub z: i16,
}

impl Gyro {
    pub(crate) fn from_bytes(data: [u8; 6]) -> Self {
        let x = [data[0], data[1]];
        let y = [data[2], data[3]];
        let z = [data[4], data[5]];
        Self {
            x: i16::from_be_bytes(x),
            y: i16::from_be_bytes(y),
            z: i16::from_be_bytes(z),
        }
    }
}
