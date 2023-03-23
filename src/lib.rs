#![no_std]
#![feature(strict_provenance)]
#![deny(missing_docs)]
// #![deny(warnings)]
#![deny(unused_import_braces)]
#![deny(unused_results)]
#![deny(trivial_casts)]
#![deny(trivial_numeric_casts)]
#![deny(unused_qualifications)]
// don't want this to show up in pedantic for now
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::struct_excessive_bools)]
#![allow(clippy::unused_self)]

//! # tudelft quadrupel support library
//!
//! This library re-exports some crates (see below). That just makes it
//! easy for your drone code to use those libraries. For example:
//! ```
//! use tudelft_quadrupel::cortex_m;
//! ```

extern crate alloc;
/// reexport of the `nrf51_hal` crate
pub extern crate nrf51_hal;

/// reexport of the `cortex_m_rt` entry macro
pub use cortex_m_rt::entry;
/// reexport of the `nb::block` macro.
pub use nb::block;

/// reexport of the `cortex_m` crate
pub use cortex_m;
/// reexport of the `cortex_m_rt` crate
pub use cortex_m_rt;
/// reexport of the `fixed` crate
pub use fixed;
/// reexport of the `nrf51_pac` crate
pub use nrf51_pac;
/// reexport of the `ringbuffer` crate
pub use ringbuffer;

/// Utilities to read out the barometer
pub mod barometer;

/// Utilities to read out the battery voltage. You may see
/// this referred to as the "ADC" (analog to digital converter).
pub mod battery;

/// Utilities to read from and write to the flash chip
pub mod flash;

/// Initialize all the drivers
pub mod initialize;

/// Utilities to control the leds on the board.
///
/// Note that in the template,
/// some leds have already been assigned meaning:
///
/// * red blinking: you probably have a panic
/// * blue blinking: your code is probably running fine
/// * yellow on + red blinking: this happens during initialization. If initialization fails,
///   this is never turned off. If a panic happens and yellow is on, initialization likely failed
/// * green on + red blinking: an allocation happened causing a panic.
///
/// You are free to change the meaning of these leds (though some restrictions apply, see the assignment manual).
pub mod led;

/// Utilities to drive the drone motors (PWM control)
pub mod motor;

/// Utilities to read out the motion processing unit (mpu)
pub mod mpu;

/// A [`Mutex`](mutex::Mutex) abstraction like you learned in Software Systems. Turns off interrupts
pub mod mutex;

/// A [`OnceCell`](once_cell::OnceCell) abstraction like you learned in Software Systems
pub mod once_cell;

/// Utilities to read out the current time
pub mod time;

/// Utilities to read from and write to UART
pub mod uart;

/// Internal utilities to read out TWI (I2C) devices
mod twi;
