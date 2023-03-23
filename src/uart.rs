use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
use cortex_m::peripheral::NVIC;
use nrf51_pac::interrupt;
use ringbuffer::{ConstGenericRingBuffer, RingBuffer, RingBufferRead, RingBufferWrite};

const BUFFER_SIZE: usize = 256;

/// Can be used for interfacing with the UART.
/// It uses an interrupt to send bytes, when they're ready to send.
struct UartDriver {
    rx_buffer: ConstGenericRingBuffer<u8, BUFFER_SIZE>,
    tx_buffer: ConstGenericRingBuffer<u8, BUFFER_SIZE>,
    tx_data_available: bool,
    uart: nrf51_pac::UART0,
}

static UART: Mutex<OnceCell<UartDriver>> = Mutex::new(OnceCell::uninitialized());

pub(crate) fn initialize(uart: nrf51_pac::UART0, nvic: &mut NVIC) {
    // In this function the following things are done:
    // 1. Enable the UART peripheral
    uart.baudrate.write(|w| w.baudrate().baud115200());
    uart.enable.write(|w| w.enable().enabled());
    // 2. Configure the UART peripheral
    uart.events_rxdrdy.reset();
    uart.events_txdrdy.reset();
    uart.events_error.reset();

    uart.tasks_starttx.write(|w| unsafe { w.bits(1) });
    uart.tasks_startrx.write(|w| unsafe { w.bits(1) });
    // 3. Configure the UART interrupt
    uart.intenclr.write(|w| unsafe { w.bits(u32::MAX) });
    uart.intenset
        .write(|w| w.rxdrdy().set_bit().txdrdy().set_bit().error().set_bit());

    NVIC::unpend(nrf51_pac::Interrupt::UART0);
    // 5. Set the interrupt priority
    // SAFETY: only unsafe because changing interrupts means critical sections
    // are required. However, since we use critical sections everywhere (and no
    // priority-based crical sections) this is safe.
    unsafe { nvic.set_priority(nrf51_pac::Interrupt::UART0, 3) };

    // actualy enable the uart interrupt
    unsafe { NVIC::unmask(nrf51_pac::Interrupt::UART0) };

    UART.modify(|uartd| {
        uartd.initialize(UartDriver {
            rx_buffer: ConstGenericRingBuffer::default(),
            tx_buffer: ConstGenericRingBuffer::default(),
            tx_data_available: true,
            uart,
        })
    })
}

/// Checks if the UART is initialized
pub fn is_initialized() -> bool {
    UART.modify(|uart| uart.is_initialized())
}

/// Safe usage: this should only be called if you never run any real code after this again.
/// That's because there is *no way* to reinitialize the UART. This is only so you can make
/// it so an allocation error in the panic handler doesn't cause the allocation error to happen
/// again
#[doc(hidden)]
pub unsafe fn uninitialize() {
    UART.modify(|uart| uart.uninitialize())
}

/// Reads as many bytes as possible from the UART
pub fn receive_bytes(bytes: &mut [u8]) -> usize {
    UART.modify(|uart| {
        let mut i = 0;
        while let Some(byte) = get_byte(uart) {
            bytes[i] = byte;
            i += 1;
            if i == bytes.len() {
                break;
            }
        }
        i
    })
}

/// Reads a single byte from the UART
fn get_byte(uart: &mut OnceCell<UartDriver>) -> Option<u8> {
    uart.rx_buffer.dequeue()
}

/// Writes the entire buffer over UART
pub fn send_bytes(bytes: &[u8]) -> bool {
    UART.modify(|uart| {
        if uart.tx_buffer.len() + bytes.len() >= uart.tx_buffer.capacity() {
            return false;
        }

        for byte in bytes {
            put_byte(uart, *byte);
        }

        true
    })
}

/// Pushes a single byte over uart
fn put_byte(uart: &mut OnceCell<UartDriver>, byte: u8) {
    if uart.tx_data_available {
        uart.tx_data_available = false;
        uart.uart.txd.write(|w| unsafe { w.txd().bits(byte) });
    } else {
        uart.tx_buffer.enqueue(byte);
    }
}

#[interrupt]
/// Interrupt handler for UART0
/// It's called when the enabled interrupts for uart0 are triggered
unsafe fn UART0() {
    // Safety: interrupts are already turned off here, since we are inside an interrupt
    let uart = unsafe { UART.no_critical_section_lock_mut() };

    if uart.uart.events_rxdrdy.read().bits() != 0 {
        uart.uart.events_rxdrdy.reset();
        let byte = uart.uart.rxd.read().rxd().bits();
        uart.rx_buffer.enqueue(byte);
    }

    if uart.uart.events_txdrdy.read().bits() != 0 {
        uart.uart.events_txdrdy.reset();
        if let Some(byte) = uart.tx_buffer.dequeue() {
            uart.uart.txd.write(|w| unsafe { w.txd().bits(byte) });
        } else {
            uart.tx_data_available = true;
        }
    }

    if uart.uart.events_error.read().bits() != 0 {
        uart.uart.events_error.reset();
    }
}
