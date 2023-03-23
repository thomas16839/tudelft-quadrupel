use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
use core::sync::atomic::{AtomicBool, Ordering};
use cortex_m::peripheral::NVIC;
use nrf51_hal::gpio::p0::{P0_02, P0_04};
use nrf51_hal::gpio::{Disconnected, Pin};
use nrf51_pac::interrupt;
use nrf51_pac::twi0::frequency::FREQUENCY_A;
use nrf51_pac::{Interrupt, GPIO, TWI0};

const FREQ: FREQUENCY_A = FREQUENCY_A::K400;

pub(crate) static TWI: Mutex<OnceCell<TwiWrapper>> = Mutex::new(OnceCell::uninitialized());

pub struct TwiWrapper {
    twi: TWI0,
    sent: AtomicBool,
    recv: AtomicBool,
}

pub enum TwiStatus {
    BufEmpty,
    Success,
}

impl TwiWrapper {
    fn set_sent_flag(&self, value: bool) {
        self.sent.store(value, Ordering::SeqCst)
    }

    fn set_recv_flag(&self, value: bool) {
        self.recv.store(value, Ordering::SeqCst)
    }

    fn wait(&self, flag: &AtomicBool) {
        while !flag.load(Ordering::SeqCst) {
            core::hint::spin_loop();
        }
    }

    fn wait_sent(&self) {
        self.wait(&self.sent)
    }

    fn wait_recv(&self) {
        self.wait(&self.recv)
    }

    pub fn read(&self, addr: u8, reg_addr: u8, data: &mut [u8]) -> TwiStatus {
        if data.is_empty() {
            return TwiStatus::BufEmpty;
        }

        self.set_sent_flag(false);
        self.set_recv_flag(false);

        self.twi
            .address
            .write(|w| unsafe { w.address().bits(addr) });
        self.twi.txd.write(|w| unsafe { w.txd().bits(reg_addr) });
        self.twi.shorts.reset();
        self.twi.tasks_starttx.write(|w| unsafe { w.bits(1) });

        self.wait_sent();
        self.set_sent_flag(false);

        if data.len() == 1 {
            self.twi.shorts.write(|w| w.bb_stop().set_bit())
        } else {
            self.twi.shorts.write(|w| w.bb_suspend().set_bit())
        }

        self.twi.tasks_startrx.write(|w| unsafe { w.bits(1) });

        let mut bytes_left = data.len();
        let mut write_ptr = 0;

        loop {
            self.wait_recv();
            self.set_recv_flag(false);

            data[write_ptr] = self.twi.rxd.read().rxd().bits();
            write_ptr += 1;

            bytes_left -= 1;
            if bytes_left == 1 {
                self.twi.shorts.write(|w| w.bb_stop().set_bit())
            }
            self.twi.tasks_resume.write(|w| unsafe { w.bits(1) });

            if bytes_left == 0 {
                break;
            }
        }

        TwiStatus::Success
    }

    pub fn write(&self, addr: u8, reg_addr: u8, data: &[u8]) {
        if data.is_empty() {
            self.twi
                .address
                .write(|w| unsafe { w.address().bits(addr) });
            self.twi.shorts.write(|w| w.bb_stop().set_bit());
            self.twi.txd.write(|w| unsafe { w.txd().bits(reg_addr) });
            self.twi.tasks_starttx.write(|w| unsafe { w.bits(1) });
            self.wait_sent();

            return;
        }

        self.set_sent_flag(false);

        self.twi
            .address
            .write(|w| unsafe { w.address().bits(addr) });
        self.twi.shorts.reset();
        self.twi.txd.write(|w| unsafe { w.txd().bits(reg_addr) });
        self.twi.tasks_starttx.write(|w| unsafe { w.bits(1) });

        self.wait_sent();
        self.set_sent_flag(false);

        for &i in data {
            self.twi.txd.write(|w| unsafe { w.txd().bits(i) });

            self.wait_sent();
            self.set_sent_flag(false);
        }

        self.twi.tasks_stop.write(|w| unsafe { w.bits(1) });
    }
}

pub(crate) fn initialize(
    twi: TWI0,
    scl_pin: P0_04<Disconnected>,
    sda_pin: P0_02<Disconnected>,
    nvic: &mut NVIC,
) {
    let scl_pin = Pin::from(scl_pin.into_pullup_input());
    let sda_pin = Pin::from(sda_pin.into_pullup_input());

    // The TWIM peripheral requires the pins to be in a mode that is not
    // exposed through the GPIO API, and might it might not make sense to
    // expose it there.
    //
    // Until we've figured out what to do about this, let's just configure
    // the pins through the raw peripheral API. All of the following is
    // safe, as we own the pins now and have exclusive access to their
    // registers.
    for &pin in &[scl_pin.pin(), sda_pin.pin()] {
        unsafe { &*GPIO::ptr() }.pin_cnf[pin as usize].write(|w| {
            w.dir()
                .input()
                .input()
                .connect()
                .pull()
                .pullup()
                .drive()
                .s0d1()
                .sense()
                .disabled()
        });
    }

    // Set pins.
    twi.pselscl
        .write(|w| unsafe { w.bits(scl_pin.pin().into()) });
    twi.pselsda
        .write(|w| unsafe { w.bits(sda_pin.pin().into()) });

    // Clear interrupts
    twi.events_rxdready.reset();
    twi.events_txdsent.reset();

    // Set frequency.
    twi.frequency.write(|w| w.frequency().variant(FREQ));

    // Set which interrupts we want to receive
    twi.intenset
        .write(|w| w.txdsent().set_bit().rxdready().set_bit().error().set_bit());

    twi.shorts.reset();
    twi.enable.write(|w| w.enable().enabled());

    // Initialize oncecell
    // twi.events_rxdready.reset();
    // twi.events_txdsent.reset();
    TWI.modify(|t| {
        t.initialize(TwiWrapper {
            twi,
            recv: false.into(),
            sent: false.into(),
        })
    });

    // Setup NVIC
    NVIC::unpend(Interrupt::SPI0_TWI0);
    // Safety: We are not using priority-based critical sections.
    unsafe {
        nvic.set_priority(Interrupt::SPI0_TWI0, 3); // Same as C template
        NVIC::unmask(Interrupt::SPI0_TWI0);
    }
}

#[interrupt]
unsafe fn SPI0_TWI0() {
    // Safety: interrupts are already turned off here, since we are inside an interrupt
    // We might be accessing the hardware while the interrupted code also wants to, this is fine since we're only touching the EVENT registers which are not touched by the other code
    let twi = unsafe { TWI.no_critical_section_lock_mut() };

    if twi.twi.events_rxdready.read().bits() != 0 {
        twi.twi.events_rxdready.reset();
        twi.recv.store(true, Ordering::SeqCst);
    }
    if twi.twi.events_txdsent.read().bits() != 0 {
        twi.twi.events_txdsent.reset();
        twi.sent.store(true, Ordering::SeqCst);
    }

    // Errors are silently ignored
    if twi.twi.events_error.read().bits() != 0 {
        twi.twi
            .errorsrc
            .write(|w| w.anack().clear_bit().overrun().clear_bit()); // Clear error source
        twi.twi.events_error.reset();
    }
}
