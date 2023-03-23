use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
use cortex_m::interrupt::free;
use nrf51_hal::gpio::p0::{P0_22, P0_24, P0_28, P0_30};
use nrf51_hal::gpio::{Disconnected, Level, Output, PushPull};
use nrf51_hal::prelude::{OutputPin, StatefulOutputPin};
use void::ResultVoidExt;

pub use Led::{Blue, Green, Red, Yellow};

/// There are four leds on the control board, these can be accessed using this enum.
///
/// For example, to toggle the red led use
/// ```no_run
/// use tudelft_quadrupel::led::Red;
///
/// Red.toggle();
/// ```
#[derive(Copy, Clone)]
pub enum Led {
    /// This controls the red led
    Red,
    /// This controls the red yellow
    Yellow,
    /// This controls the red green
    Green,
    /// This controls the red blue
    Blue,
}

impl Led {
    /// Turns the selected led off. If the led was already off, nothing changes
    pub fn off(self) {
        // SAFETY: we don't need a critical section here since writing
        // to leds is an atomic operation already. This is as from the
        // inline comments from the `nrf_51` crate on `set_high`.
        //
        // NOTE: single writes on ARM are always atomic so this makes sense.
        let leds = unsafe { LEDS.no_critical_section_lock_mut() };

        // ignore the error here. Its type is `Void` and is impossible
        // to construct (and thus impossible to actually happen)
        let _ = match self {
            Red => leds.led_red.set_high(),
            Yellow => leds.led_yellow.set_high(),
            Green => leds.led_green.set_high(),
            Blue => leds.led_blue.set_high(),
        };
    }

    /// Turns the selected led on. If the led was already on, nothing changes
    pub fn on(self) {
        // SAFETY: we don't need a critical section here since writing
        // to leds is an atomic operation already. This is as from the
        // inline comments from the `nrf_51` crate on `set_low`.
        //
        // NOTE: single writes on ARM are always atomic so this makes sense.
        let leds = unsafe { LEDS.no_critical_section_lock_mut() };

        // ignore the error here. Its type is `Void` and is impossible
        // to construct (and thus impossible to actually happen)
        let _ = match self {
            Red => leds.led_red.set_low(),
            Yellow => leds.led_yellow.set_low(),
            Green => leds.led_green.set_low(),
            Blue => leds.led_blue.set_low(),
        };
    }

    /// Checks whether a led is off or not. Returns true if the led was on.
    pub fn is_off(self) -> bool {
        // SAFETY: we don't need a critical section here since reading from
        // leds is an atomic operation already. This is as from the
        // inline comments from the `nrf_51` crate on `is_set_high`.
        //
        // NOTE: single writes on ARM are always atomic so this makes sense.
        let leds = unsafe { LEDS.no_critical_section_lock_mut() };

        let res = match self {
            Red => leds.led_red.is_set_high(),
            Yellow => leds.led_yellow.is_set_high(),
            Green => leds.led_green.is_set_high(),
            Blue => leds.led_blue.is_set_high(),
        };

        res.void_unwrap()
    }

    /// Sets the state of a led. true = on, false = off
    pub fn set(self, value: bool) {
        if value {
            self.on();
        } else {
            self.off();
        }
    }

    /// Checks whether a led is on or not. Returns true if the led was on.
    #[must_use]
    pub fn is_on(self) -> bool {
        !self.is_off()
    }

    /// Toggle this led. If it was on, it now turns off. If it was off, it now turns on.
    /// Returns the state of the led after toggling true = on, false = off.
    pub fn toggle(self) -> bool {
        // create a critical section here, so an interrupt cannot occur
        // between is_enabled and the actual enable call. This immediately
        // makes the unsafe `no_critical_section_lock` in `is_enabled`, `disable`
        // and `enable` guaranteed to be safe.
        free(|_| {
            if self.is_on() {
                self.off();
                false
            } else {
                self.on();
                true
            }
        })
    }
}

struct Leds {
    pub led_red: P0_22<Output<PushPull>>,
    pub led_yellow: P0_24<Output<PushPull>>,
    pub led_green: P0_28<Output<PushPull>>,
    pub led_blue: P0_30<Output<PushPull>>,
}

static LEDS: Mutex<OnceCell<Leds>> = Mutex::new(OnceCell::uninitialized());

pub(crate) fn initialize(
    led_red: P0_22<Disconnected>,
    led_yellow: P0_24<Disconnected>,
    led_green: P0_28<Disconnected>,
    led_blue: P0_30<Disconnected>,
) {
    LEDS.modify(|leds| {
        leds.initialize(Leds {
            led_red: led_red.into_push_pull_output(Level::High),
            led_yellow: led_yellow.into_push_pull_output(Level::High),
            led_green: led_green.into_push_pull_output(Level::High),
            led_blue: led_blue.into_push_pull_output(Level::High),
        })
    });
}
