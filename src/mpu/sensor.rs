use crate::led::Green;
use crate::mpu::config::Fifo;
use crate::mpu::config::GyroFullScale;
use crate::mpu::config::{AccelFullScale, ClockSource, DigitalLowPassFilter};
use crate::mpu::registers::Register;
use crate::mpu::structs::{Accel, Gyro};
use crate::time::delay_ms_assembly;
use crate::twi::TwiWrapper;
use core::marker::PhantomData;
use core::time::Duration;
use embedded_hal::blocking::i2c::{Write, WriteRead};

const MPU6050_ADDRESS: u8 = 0x68;

pub type I2c = TwiWrapper;

pub(crate) struct Mpu6050(PhantomData<()>);

impl Mpu6050 {
    /// Construct a new i2c driver for the MPU-6050
    pub fn new(i2c: &mut I2c) -> Self {
        let mut sensor = Self(PhantomData::default());

        sensor.disable_sleep(i2c);

        sensor
    }

    /// Load DMP firmware and perform all appropriate initialization.
    pub fn initialize_dmp(&mut self, i2c: &mut I2c) {
        self.reset(i2c);
        self.disable_sleep(i2c);
        self.reset_signal_path(i2c);
        self.disable_dmp(i2c);
        self.set_clock_source(i2c, ClockSource::Xgyro);
        self.disable_interrupts(i2c);
        self.set_fifo_enabled(i2c, Fifo::all_disabled());
        self.set_accel_full_scale(i2c, AccelFullScale::G2);
        self.set_sample_rate_divider(i2c, 0);
        self.set_digital_lowpass_filter(i2c, DigitalLowPassFilter::Filter0);
        self.load_firmware(i2c);
        self.boot_firmware(i2c);
        self.set_gyro_full_scale(i2c, GyroFullScale::Deg2000);
        self.enable_fifo(i2c);
        self.reset_fifo(i2c);
        self.disable_dmp(i2c);
        self.enable_dmp(i2c);
    }

    pub(crate) fn read(&mut self, i2c: &mut I2c, reg: u8, response: &mut [u8]) {
        let _ = i2c.read(MPU6050_ADDRESS, reg, response);
    }

    pub(crate) fn write(&mut self, i2c: &mut I2c, reg_address: u8, bytes: &[u8]) {
        i2c.write(MPU6050_ADDRESS, reg_address, bytes);
    }

    pub(crate) fn read_register(&mut self, i2c: &mut I2c, reg: Register) -> u8 {
        let mut buf = [0; 1];
        self.read(i2c, reg as u8, &mut buf);
        buf[0]
    }

    pub(crate) fn read_registers<'a>(
        &mut self,
        i2c: &mut I2c,
        reg: Register,
        buf: &'a mut [u8],
    ) -> &'a [u8] {
        self.read(i2c, reg as u8, buf);
        buf
    }

    pub(crate) fn write_register(&mut self, i2c: &mut I2c, reg: Register, value: u8) {
        self.write(i2c, reg as u8, &[value]);
    }

    // ------------------------------------------------------------------------
    // ------------------------------------------------------------------------

    /// Perform power reset of the MPU
    pub fn reset(&mut self, i2c: &mut I2c) {
        let mut value = self.read_register(i2c, Register::PwrMgmt1);
        value |= 1 << 7;
        self.write_register(i2c, Register::PwrMgmt1, value);
        delay_ms_assembly(200);
    }

    /// Perform reset of the signal path
    pub fn reset_signal_path(&mut self, i2c: &mut I2c) {
        let mut value = self.read_register(i2c, Register::UserCtrl);
        value |= 1 << 0;
        self.write_register(i2c, Register::UserCtrl, value);
        delay_ms_assembly(200);
    }

    /// Pick the clock-source
    pub fn set_clock_source(&mut self, i2c: &mut I2c, clock_source: ClockSource) {
        let mut value = self.read_register(i2c, Register::PwrMgmt1);
        value |= clock_source as u8;
        self.write_register(i2c, Register::PwrMgmt1, value);
    }

    pub fn disable_interrupts(&mut self, i2c: &mut I2c) {
        self.write_register(i2c, Register::IntEnable, 0x00)
    }

    pub fn set_accel_full_scale(&mut self, i2c: &mut I2c, scale: AccelFullScale) {
        let mut value = self.read_register(i2c, Register::AccelConfig);
        value |= (scale as u8) << 3;
        self.write_register(i2c, Register::AccelConfig, value)
    }

    pub fn set_gyro_full_scale(&mut self, i2c: &mut I2c, scale: GyroFullScale) {
        let mut value = self.read_register(i2c, Register::GyroConfig);
        value |= (scale as u8) << 3;
        self.write_register(i2c, Register::GyroConfig, value)
    }

    pub fn set_sample_rate_divider(&mut self, i2c: &mut I2c, div: u8) {
        self.write_register(i2c, Register::SmpRtDiv, div)
    }

    pub fn set_digital_lowpass_filter(&mut self, i2c: &mut I2c, filter: DigitalLowPassFilter) {
        let mut value = self.read_register(i2c, Register::Config);
        value |= filter as u8;
        self.write_register(i2c, Register::Config, value)
    }

    pub fn reset_fifo(&mut self, i2c: &mut I2c) {
        let mut value = self.read_register(i2c, Register::UserCtrl);
        value |= 1 << 2;
        self.write_register(i2c, Register::UserCtrl, value)
    }

    pub fn enable_fifo(&mut self, i2c: &mut I2c) {
        let mut value = self.read_register(i2c, Register::UserCtrl);
        value |= 1 << 6;
        self.write_register(i2c, Register::UserCtrl, value)
    }

    pub fn disable_fifo(&mut self, i2c: &mut I2c) {
        let mut value = self.read_register(i2c, Register::UserCtrl);
        value &= !(1 << 6);
        self.write_register(i2c, Register::UserCtrl, value)
    }

    /// Set the DMP bit.
    /// To perform full DMP initialization, see `initialize_dmp()`
    pub fn enable_dmp(&mut self, i2c: &mut I2c) {
        let mut value = self.read_register(i2c, Register::UserCtrl);
        value |= 1 << 7;
        self.write_register(i2c, Register::UserCtrl, value)
    }

    // Unset the DMP bit.
    pub fn disable_dmp(&mut self, i2c: &mut I2c) {
        let mut value = self.read_register(i2c, Register::UserCtrl);
        value &= !(1 << 7);
        self.write_register(i2c, Register::UserCtrl, value)
    }

    /// Reset the DMP processor
    pub fn reset_dmp(&mut self, i2c: &mut I2c) {
        let mut value = self.read_register(i2c, Register::UserCtrl);
        value |= 1 << 3;
        self.write_register(i2c, Register::UserCtrl, value)
    }

    /// Read the FIFO
    pub fn read_fifo<'a>(&mut self, i2c: &mut I2c, buf: &'a mut [u8]) -> &'a [u8] {
        self.read_registers(i2c, Register::FifoRw, &mut buf[..])
    }

    pub fn get_fifo_enabled(&mut self, i2c: &mut I2c) -> Fifo {
        let value = self.read_register(i2c, Register::FifoEn);
        Fifo::from_byte(value)
    }

    pub fn set_fifo_enabled(&mut self, i2c: &mut I2c, fifo: Fifo) {
        self.write_register(i2c, Register::FifoEn, fifo.to_byte())
    }

    pub fn get_fifo_count(&mut self, i2c: &mut I2c) -> usize {
        let mut buf = [0; 2];
        let _value = self.read_registers(i2c, Register::FifoCount_H, &mut buf);
        u16::from_be_bytes(buf) as usize
    }

    pub fn disable_sleep(&mut self, i2c: &mut I2c) {
        let mut value = self.read_register(i2c, Register::PwrMgmt1);
        value &= !(1 << 6);
        self.write_register(i2c, Register::PwrMgmt1, value)
    }

    pub fn accel(&mut self, i2c: &mut I2c) -> Accel {
        let mut data = [0; 6];
        let _ = self.read_registers(i2c, Register::AccelX_H, &mut data);
        Accel::from_bytes(data)
    }

    pub fn gyro(&mut self, i2c: &mut I2c) -> Gyro {
        let mut data = [0; 6];
        let _ = self.read_registers(i2c, Register::GyroX_H, &mut data);
        Gyro::from_bytes(data)
    }
}
