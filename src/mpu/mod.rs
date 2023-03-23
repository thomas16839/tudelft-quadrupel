use crate::mpu::config::DigitalLowPassFilter;
use crate::mpu::sensor::Mpu6050;
use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
use crate::twi::{TwiWrapper, TWI};
use error::Error;
use nb::Error::WouldBlock;
use structs::{Accel, Gyro, Quaternion};

#[allow(unused)]
mod config;
mod dmp_firmware;
mod firmware_loader;
#[allow(unused)]
mod registers;
#[allow(unused)]
mod sensor;
/// structs to deal with mpu output, like quaternions
pub mod structs;

mod error;

/// MPU Sample Rate Divider under DMP mode
pub const SAMPLE_RATE_DIVIDER_MPU: u8 = 0;
/// MPU Sample Rate Divider under RAW mode
pub const SAMPLE_RATE_DIVIDER_RAW: u8 = 0;

type I2c = TwiWrapper;

struct Mpu {
    mpu: Mpu6050,
    dmp_enabled: bool,
}

static MPU: Mutex<OnceCell<Mpu>> = Mutex::new(OnceCell::uninitialized());

pub(crate) fn initialize() {
    // Safety: The TWI mutex is not accessed in an interrupt
    let twi = unsafe { TWI.no_critical_section_lock_mut() };

    let mut mpu = Mpu6050::new(twi);

    mpu.initialize_dmp(twi);

    mpu.set_sample_rate_divider(twi, SAMPLE_RATE_DIVIDER_MPU);
    mpu.set_digital_lowpass_filter(twi, DigitalLowPassFilter::Filter5);
    MPU.modify(|m| {
        m.initialize(Mpu {
            mpu,
            dmp_enabled: true,
        })
    });
}

/// Is the DMP (digital motion processor) of the MPU enabled?
/// It is enabled by default.
pub fn is_dmp_enabled() -> bool {
    MPU.modify(|mpu| mpu.dmp_enabled)
}

/// Disable the DMP (digital motion processor) of the MPU
///
/// # Panics
/// when the global constant `SAMPLE_RATE_DIVIDER_RAW` is wrong (i.e. won't panic under normal conditions)
pub fn disable_dmp() {
    // Safety: The TWI and MPU mutexes are not accessed in an interrupt
    let twi = unsafe { TWI.no_critical_section_lock_mut() };
    let mpu = unsafe { MPU.no_critical_section_lock_mut() };

    mpu.mpu
        .set_sample_rate_divider(twi, SAMPLE_RATE_DIVIDER_RAW);
    mpu.mpu.disable_dmp(twi);
    mpu.mpu.disable_fifo(twi);
    mpu.dmp_enabled = false;
}

/// Enable the DMP (digital motion processor) of the MPU
///
/// # Errors
/// when the global constant `SAMPLE_RATE_DIVIDER_MPU` is wrong (i.e. will not panic under normal conditions)
pub fn enable_dmp() {
    // Safety: The TWI and MPU mutexes are not accessed in an interrupt
    let twi = unsafe { TWI.no_critical_section_lock_mut() };
    let mpu = unsafe { MPU.no_critical_section_lock_mut() };

    mpu.mpu
        .set_sample_rate_divider(twi, SAMPLE_RATE_DIVIDER_MPU);
    mpu.mpu.enable_dmp(twi);
    mpu.mpu.enable_fifo(twi);
    mpu.dmp_enabled = true;
}

/// This reads the most recent angle from the DMP, if there are any new ones available.
/// If there is no new angle available, it returns `WouldBlock`.
/// Do not call this function if the DMP is disabled.
///
/// # Panics
/// When the dmp is disabled.
///
/// # Errors
/// when a TWI(I2C) operation failed
pub fn read_dmp_bytes() -> nb::Result<Quaternion, ()> {
    // Safety: The TWI and MPU mutexes are not accessed in an interrupt
    let twi = unsafe { TWI.no_critical_section_lock_mut() };
    let mpu = unsafe { MPU.no_critical_section_lock_mut() };

    assert!(mpu.dmp_enabled);

    // If there isn't a full packet ready, return none
    let mut len = mpu.mpu.get_fifo_count(twi);
    if len < 28 {
        return Err(WouldBlock);
    }

    // If we got mis-aligned, we skip a packet
    if len % 28 != 0 {
        let skip = len % 28;
        let mut buf = [0; 28];

        let _ = mpu.mpu.read_fifo(twi, &mut buf[..skip]);
        return Err(WouldBlock);
    }

    // Keep reading while there are more full packets
    let mut buf = [0; 28];
    while len >= 28 {
        let _ = mpu.mpu.read_fifo(twi, &mut buf);
        len -= 28;
    }

    // Convert the last full packet we received to a Quaternion
    Ok(Quaternion::from_bytes(&buf[..16]))
}

/// This reads the most recent acceleration and gyroscope information from the MPU.
/// This function can be called both if the DMP is enabled or disabled.
///
/// # Errors
/// when a TWI operation failed
pub fn read_raw() -> Result<(Accel, Gyro), Error> {
    // Safety: The TWI and MPU mutexes are not accessed in an interrupt
    let twi = unsafe { TWI.no_critical_section_lock_mut() };
    let mpu = unsafe { MPU.no_critical_section_lock_mut() };

    let accel = mpu.mpu.accel(twi);
    let gyro = mpu.mpu.gyro(twi);

    Ok((accel, gyro))
}
