use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
use crate::time::{delay_ms_assembly, delay_us_assembly};
use nb::block;
use nrf51_hal::gpio::p0::{P0_00, P0_09, P0_11, P0_13, P0_17, P0_18};
use nrf51_hal::gpio::Level;
use nrf51_hal::gpio::{Disconnected, Output, PushPull};
use nrf51_hal::prelude::OutputPin;
use nrf51_hal::spi::{Frequency, Pins};
use nrf51_hal::spi::{FullDuplex, MODE_0};
use nrf51_hal::Spi;
use nrf51_pac::SPI1;

const WRSR: u8 = 0x01;
const BYTEWRITE: u8 = 0x02;
const BYTEREAD: u8 = 0x03;
const WRDI: u8 = 0x04;
// const RDSR: u8 = 0x05;
const WREN: u8 = 0x06;
const EWSR: u8 = 0x50;
const CHIP_ERASE: u8 = 0x60;
const AAI: u8 = 0xAF;

static FLASH: Mutex<OnceCell<SpiFlash>> = Mutex::new(OnceCell::uninitialized());

/// Errors that may occur while interacting with the flash chip
#[derive(Debug)]
pub enum FlashError {
    /// Writing over spi failed.
    SpiError(nrf51_hal::spi::Error),
    /// When a buffer is written at a location too close to the end of flash
    /// this error is raised
    OutOfSpace,
}

impl From<void::Void> for FlashError {
    fn from(v: void::Void) -> Self {
        match v {}
    }
}

impl From<nrf51_hal::spi::Error> for FlashError {
    fn from(e: nrf51_hal::spi::Error) -> Self {
        FlashError::SpiError(e)
    }
}

struct SpiFlash {
    spi: Spi<SPI1>,
    _pin_wp: P0_00<Output<PushPull>>,
    _pin_hold: P0_13<Output<PushPull>>,
    pin_cs: P0_17<Output<PushPull>>,
}

/// Initialize the flash memory. Should be called only once.
pub(crate) fn initialize(
    spi1: SPI1,
    pin_cs: P0_17<Disconnected>,
    pin_miso: P0_18<Disconnected>,
    pin_wp: P0_00<Disconnected>,
    pin_hold: P0_13<Disconnected>,
    pin_sck: P0_11<Disconnected>,
    pin_mosi: P0_09<Disconnected>,
) -> Result<(), FlashError> {
    let spi = Spi::new(
        spi1,
        Pins {
            sck: Some(pin_sck.into_push_pull_output(Level::Low).degrade()),
            mosi: Some(pin_mosi.into_push_pull_output(Level::Low).degrade()),
            miso: Some(pin_miso.into_floating_input().degrade()),
        },
        Frequency::M4,
        MODE_0,
    );
    let pin_wp = pin_wp.into_push_pull_output(Level::High);
    let pin_hold = pin_hold.into_push_pull_output(Level::High);
    let pin_cs = pin_cs.into_push_pull_output(Level::High);

    FLASH.modify(|spi_flash| {
        spi_flash.initialize(SpiFlash {
            spi,
            _pin_wp: pin_wp,
            _pin_hold: pin_hold,
            pin_cs,
        });
    });

    flash_enable_wsr()?;
    flash_set_wrsr()?;
    flash_chip_erase()?;
    flash_write_enable()?;
    Ok(())
}

/// Transmit data over SPI. Ignore any received data.
fn spi_master_tx(tx_data: &[u8]) -> Result<(), FlashError> {
    assert_ne!(tx_data.len(), 0);

    // Safety: The FLASH mutex is not accessed in an interrupt
    let guard = unsafe { FLASH.no_critical_section_lock_mut() };

    // Enable slave
    guard.pin_cs.set_low()?;

    block!(guard.spi.send(tx_data[0]))?;
    for i in 0..tx_data.len() - 1 {
        block!(guard.spi.send(tx_data[i + 1]))?;
        let _ = block!(guard.spi.read())?;
    }
    let _ = block!(guard.spi.read())?;

    // Disable slave
    guard.pin_cs.set_high()?;
    Ok(())
}

/// Transmit data over SPI. Optimized to read bytes from the flash memory.
fn spi_master_tx_rx_fast_read(tx_data: [u8; 4], rx_data: &mut [u8]) -> Result<(), FlashError> {
    assert_ne!(rx_data.len(), 0);

    // Safety: The FLASH mutex is not accessed in an interrupt
    let guard = unsafe { FLASH.no_critical_section_lock_mut() };

    // Enable slave
    guard.pin_cs.set_low()?;

    for byte in tx_data {
        block!(guard.spi.send(byte))?;
        let _ = block!(guard.spi.read())?;
    }

    for byte in rx_data {
        block!(guard.spi.send(0))?;
        *byte = block!(guard.spi.read())?;
    }

    // Disable slave
    guard.pin_cs.set_high()?;

    Ok(())
}

/// Transmit data over SPI. Optimized to write bytes to the flash memory.
fn spi_master_tx_rx_fast_write(tx_data: [u8; 4], bytes: &[u8]) -> Result<(), FlashError> {
    assert_ne!(bytes.len(), 0);

    let mut bytes_written: u32 = 0;
    let address: u32 =
        u32::from(tx_data[3]) + (u32::from(tx_data[2]) << 8) + (u32::from(tx_data[1]) << 16);

    // Safety: The FLASH mutex is not accessed in an interrupt
    let guard = unsafe { FLASH.no_critical_section_lock_mut() };

    // Enable slave
    guard.pin_cs.set_low()?;

    for byte in tx_data {
        block!(guard.spi.send(byte))?;
        let _ = block!(guard.spi.read())?;
    }

    // Send first byte
    block!(guard.spi.send(bytes[0]))?;
    let _ = block!(guard.spi.read())?;

    // Disable slave
    guard.pin_cs.set_high()?;

    for i in 1..bytes.len() {
        delay_us_assembly(15);

        // Enable slave
        guard.pin_cs.set_low()?;
        block!(guard.spi.send(AAI))?;
        let _ = block!(guard.spi.read())?;

        block!(guard.spi.send(bytes[i]))?;
        let _ = block!(guard.spi.read())?;

        bytes_written += 1;

        // Disable slave
        guard.pin_cs.set_high()?;

        if address + bytes_written >= 0x1FFFF && i < bytes.len() - 1 {
            return Err(FlashError::OutOfSpace);
        }
    }

    delay_us_assembly(20);

    // Enable slave
    guard.pin_cs.set_low()?;

    //Send WRDI
    block!(guard.spi.send(WRDI))?;
    let _ = block!(guard.spi.read())?;

    // Disable slave
    guard.pin_cs.set_high()?;

    Ok(())
}

/// Write-Enable(WREN).
fn flash_write_enable() -> Result<(), FlashError> {
    spi_master_tx(&[WREN])
}

/// This function clears the entire flash memory.
///
/// Note: This takes about 100ms to execute, and blocks!
///
/// # Errors
/// When the SPI command fails
pub fn flash_chip_erase() -> Result<(), FlashError> {
    flash_write_enable()?;
    spi_master_tx(&[CHIP_ERASE])?;
    delay_ms_assembly(100);
    Ok(())
}

/// Enable-Write-Status-Register (EWSR). This function must be followed by `flash_enable_WSR`().
fn flash_enable_wsr() -> Result<(), FlashError> {
    spi_master_tx(&[EWSR])
}

/// Sets Write-Status-Register (WRSR) to 0x00 to enable memory write.
fn flash_set_wrsr() -> Result<(), FlashError> {
    spi_master_tx(&[WRSR, 0x00])
}

/// Writes one byte data to specified address.
///
/// Note: Make sure that the memory location is cleared before writing data. If data is already present
/// in the memory location (given address), new data cannot be written to that memory location unless
/// `flash_chip_erase`() function is called.
///
/// address: starting address (between 0x000000 to 0x01FFFF exclusive) from which the data should be stored
/// byte: one byte data to be stored at the specified address
///
/// # Errors
/// When the SPI command fails
pub fn flash_write_byte(address: u32, byte: u8) -> Result<(), FlashError> {
    flash_write_enable()?;
    spi_master_tx(&[
        BYTEWRITE,
        address.to_ne_bytes()[2],
        address.to_ne_bytes()[1],
        address.to_ne_bytes()[0],
        byte,
        0x00,
    ])?;
    delay_us_assembly(20);
    Ok(())
}

/// Writes multi-byte data into memory starting from specified address. Each memory location (address)
/// holds one byte of data.
///
/// Note: Make sure that the memory location is cleared before writing data. If data is already present
/// in the memory location (given address), new data cannot be written to that memory location unless
/// `flash_chip_erase`() function is called.
///
/// address: starting address (between 0x000000 to 0x01FFFF exclusive) from which the data should be stored
/// bytes: byte slice to write at the specified address
///
/// # Errors
/// When the SPI command fails, or the address is not within the flash address range
pub fn flash_write_bytes(address: u32, bytes: &[u8]) -> Result<(), FlashError> {
    flash_write_enable()?;
    spi_master_tx_rx_fast_write(
        [
            AAI,
            address.to_ne_bytes()[2],
            address.to_ne_bytes()[1],
            address.to_ne_bytes()[0],
        ],
        bytes,
    )?;
    Ok(())
}

/// Reads one byte data from specified address.
///
/// address: any address between 0x000000 to 0x01FFFF from where the data should be read.
///
/// # Errors
/// When the SPI command fails
pub fn flash_read_byte(address: u32) -> Result<u8, FlashError> {
    let mut rx_data = [0];
    spi_master_tx_rx_fast_read(
        [
            BYTEREAD,
            address.to_ne_bytes()[2],
            address.to_ne_bytes()[1],
            address.to_ne_bytes()[0],
        ],
        &mut rx_data,
    )?;
    Ok(rx_data[0])
}

///Reads multi-byte data starting from specified address.
///
/// address: starting address (between 0x000000 to 0x01FFFF exclusive) from which the data should be stored
/// buffer: a slice to be filled with data read from the specified location
/// # Errors
/// When the SPI command fails
///
pub fn flash_read_bytes(address: u32, buffer: &mut [u8]) -> Result<(), FlashError> {
    spi_master_tx_rx_fast_read(
        [
            BYTEREAD,
            address.to_ne_bytes()[2],
            address.to_ne_bytes()[1],
            address.to_ne_bytes()[0],
        ],
        buffer,
    )?;
    Ok(())
}
