use crate::led::Led::Yellow;
use crate::mutex::Mutex;
use crate::time::assembly_delay;
use crate::uart::send_bytes;
use crate::{barometer, battery, flash, led, motor, mpu, time, twi, uart};
use alloc_cortex_m::CortexMHeap;
use core::mem::MaybeUninit;
use nrf51_pac::Peripherals;

static INITIALIZED: Mutex<bool> = Mutex::new(false);

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

/// Initialize the drone board. This should be run at boot.
//t/
/// `heap_memory` should be a pointer to statically allocated memory.
/// Care should be taken that the mutable reference given here *really* is the
/// only mutable reference to that area of memory. That should of course be guaranteed by
/// the fact that the reference is mutable, but you need unsafe code to create a mutable
/// reference to static memory. Care should be taken that only one such reference is unsafely
/// created. The template code already does this.
///
/// when `debug` is on, the initialization process is logged over UART. However, once you have
/// a protocol in place, logging utf-8 data directly over UART is likely not desired, so you
/// will likely have to turn off `debug` at that point.
///
/// # Panics
/// This function will panic when called twice. Make sure this is only called once on boot
pub fn initialize(heap_memory: &'static mut [MaybeUninit<u8>], debug: bool) {
    // Allow time for PC to start up. The drone board starts running code immediately after upload,
    // but at that time the PC may not be listening on UART etc.
    assembly_delay(2_500_000);

    // keep this guard around until the end of the function (so interrupts stay off)
    INITIALIZED.modify(|guard| {
        assert!(!(*guard), "ALREADY INITIALIZED");
        *guard = true;
    });

    // unwrap: will never panic because this function can only be called once (see the guard above)
    let mut nrf51_peripherals = Peripherals::take().unwrap();
    let mut cortex_m_peripherals = cortex_m::Peripherals::take().unwrap();

    // Safety: `init` is safe when
    // * only called once --> the global INITIALIZED flag is set, and we panic above if called twice
    // * Heap is not empty, see the assert
    assert!(!heap_memory.is_empty());
    unsafe { ALLOCATOR.init(heap_memory.as_ptr().addr(), heap_memory.len()) }

    let gpio = nrf51_hal::gpio::p0::Parts::new(nrf51_peripherals.GPIO);
    led::initialize(gpio.p0_22, gpio.p0_24, gpio.p0_28, gpio.p0_30);
    // signal that leds have initialized
    // and that the other initialization processes are going on.
    // this also means that the processor at least booted successfully.
    Yellow.on();

    uart::initialize(nrf51_peripherals.UART0, &mut cortex_m_peripherals.NVIC);
    if debug {
        let _ = send_bytes(b"UART driver initialized\n");
    }
    time::initialize(nrf51_peripherals.RTC0, &mut cortex_m_peripherals.NVIC);
    if debug {
        let _ = send_bytes(b"RTC driver initialized\n");
    }
    twi::initialize(
        nrf51_peripherals.TWI0,
        gpio.p0_04,
        gpio.p0_02,
        &mut cortex_m_peripherals.NVIC,
    );
    if debug {
        let _ = send_bytes(b"TWI initialized\n");
    }
    mpu::initialize();
    if debug {
        let _ = send_bytes(b"MPU driver initialized\n");
    }
    barometer::initialize();
    if debug {
        let _ = send_bytes(b"Barometer driver initialized\n");
    }
    battery::initialize(nrf51_peripherals.ADC, &mut cortex_m_peripherals.NVIC);
    if debug {
        let _ = send_bytes(b"Battery driver initialized\n");
    }
    flash::initialize(
        nrf51_peripherals.SPI1,
        gpio.p0_17,
        gpio.p0_18,
        gpio.p0_00,
        gpio.p0_13,
        gpio.p0_11,
        gpio.p0_09,
    )
    .unwrap();
    if debug {
        let _ = send_bytes(b"Flash driver initialized\n");
    }
    motor::initialize(
        nrf51_peripherals.TIMER1,
        nrf51_peripherals.TIMER2,
        &mut cortex_m_peripherals.NVIC,
        &mut nrf51_peripherals.PPI,
        &mut nrf51_peripherals.GPIOTE,
    );
    if debug {
        let _ = send_bytes(b"MOTOR driver initialized\n");
    }

    // done with initialization sequence
    Yellow.off();
}
