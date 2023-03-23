use core::arch::asm;
use core::ops::Sub;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use core::time::Duration;
use nrf51_pac::interrupt;

use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
/// Delay for a number of CPU cycles. Very inaccurate
/// and hard to convert to an exact number of seconds
pub use cortex_m::asm::delay as assembly_delay;
use cortex_m::peripheral::NVIC;
use nrf51_hal::rtc::{RtcCompareReg, RtcInterrupt};
use nrf51_hal::Rtc;
use nrf51_pac::RTC0;

/// A moment in time
#[derive(Debug, Copy, Clone)]
pub struct Instant {
    time: u64,
}

impl Instant {
    /// Return the current instant, i.e. the current time
    #[must_use]
    pub fn now() -> Self {
        Self {
            time: get_time_ns(),
        }
    }

    /// Get the [`Duration`] since a previous instant. This function panics if this instant was *before* the other instant.
    ///
    /// Note: `Instant` also implements `Sub`, so you can use the minus operator instead of this function.
    ///
    /// # Panics
    /// when the `other` duration is actually in the future
    #[must_use]
    pub fn duration_since(self, other: Self) -> Duration {
        assert!(self.time >= other.time);
        Duration::from_nanos(self.time - other.time)
    }

    /// Adds a duration to this instant, producing a new instant in the future
    #[must_use]
    pub fn add_duration(self, d: Duration) -> Self {
        Self {
            time: self.time + d.as_nanos() as u64,
        }
    }

    /// Check if this `Instant` is later than another `Instant`.
    ///
    /// Note: `Instant` also implements `Ord`, so you can use the comparison operators instead of this function.
    #[must_use]
    pub fn is_later_than(self, other: Self) -> bool {
        self.time > other.time
    }

    /// Returns how many nanoseconds passed between when the timers started
    /// and the current time. This is essentially what an `Instant` inherently represents.
    #[must_use]
    pub fn ns_since_start(&self) -> u64 {
        self.time
    }
}

impl Sub<Self> for Instant {
    type Output = Duration;

    /// Get the [`Duration`] since a previous instant (rhs)
    fn sub(self, rhs: Self) -> Self::Output {
        self.duration_since(rhs)
    }
}

impl Eq for Instant {}

impl PartialEq<Self> for Instant {
    fn eq(&self, other: &Self) -> bool {
        self.time == other.time
    }
}

impl PartialOrd<Self> for Instant {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for Instant {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.time.cmp(&other.time)
    }
}

// RTC0 is used for measuring absolute instants
static RTC: Mutex<OnceCell<Rtc<RTC0>>> = Mutex::new(OnceCell::uninitialized());

/// take the highest prescaler
/// NOTE: change the period below when changing the prescaler
const PRESCALER: u32 = 0;
/// giving a period of this many nanoseconds
const PERIOD: u64 = 30517;
/// the largest value of the rtc counter before it overflows
const COUNTER_MAX: u32 = 1 << 24;

/// is set to true when the timer interrupt has gone off.
/// Used to wait on the timer interrupt in [`wait_for_next_tick`]
static TIMER_FLAG: AtomicBool = AtomicBool::new(false);

/// Global time in magic timer units ([`PERIOD`]) since timers started
/// SAFETY: only changed within timer interrupt. Safe to read at all times
static GLOBAL_TIME: Mutex<u64> = Mutex::new(0);

/// what was the counter before, so we can find the difference with what it's now.
static PREV_COUNTER: AtomicU32 = AtomicU32::new(0);

/// the number of counts until the interrupt should fire again
static COUNTER_PERIOD: AtomicU32 = AtomicU32::new(0);

pub(crate) fn initialize(clock_instance: RTC0, nvic: &mut NVIC) {
    RTC.modify(|rtc| {
        rtc.initialize(Rtc::new(clock_instance, PRESCALER).unwrap());
        rtc.enable_event(RtcInterrupt::Compare0);
        rtc.enable_interrupt(RtcInterrupt::Compare0, Some(nvic));
        rtc.enable_counter();
    });
}

// get the current global time in nanoseconds. The precision is not necessarily in single nanoseconds
fn get_time_ns() -> u64 {
    GLOBAL_TIME.modify(|global_time| {
        let counter = RTC.modify(|counter| counter.get_counter());

        // the previous state of the clock, when the global_time was last updated
        let prev_counter = PREV_COUNTER.load(Ordering::SeqCst);

        // take the global time, and add to that how much time has passed since the last interrupt
        (*global_time + u64::from(counter_diff(prev_counter, counter))) * PERIOD
    })
}

/// neatly calculates a difference in clockcycles between two rtc counter values
/// essentially a subtraction modulo `COUNTER_MAX`
fn counter_diff(prev: u32, curr: u32) -> u32 {
    if curr < prev {
        // what's left to go until the max
        // plus what we've done since 0
        (COUNTER_MAX - prev) + curr
    } else {
        curr - prev
    }
}

#[interrupt]
unsafe fn RTC0() {
    // SAFETY: we're in an interrupt so this code cannot be run concurrently anyway
    let rtc = RTC.no_critical_section_lock_mut();
    // SAFETY: we're in an interrupt so this code cannot be run concurrently anyway
    let global_time = GLOBAL_TIME.no_critical_section_lock_mut();

    if rtc.is_event_triggered(RtcInterrupt::Compare0) {
        let counter = rtc.get_counter();
        let prev_counter = PREV_COUNTER.load(Ordering::SeqCst);

        *global_time += u64::from(counter_diff(prev_counter, counter));
        PREV_COUNTER.store(counter, Ordering::SeqCst);

        let mut new_counter = counter + COUNTER_PERIOD.load(Ordering::SeqCst);
        if new_counter >= COUNTER_MAX {
            new_counter -= COUNTER_MAX;
        }

        rtc.set_compare(RtcCompareReg::Compare0, new_counter)
            .unwrap();
        rtc.reset_event(RtcInterrupt::Compare0);
        TIMER_FLAG.store(true, Ordering::SeqCst);
    }
}

/// Wait for the next interrupt configured by `set_interrupt_frequency`.
pub fn wait_for_next_tick() {
    if RTC.modify(|rtc| {
        if rtc.is_event_triggered(RtcInterrupt::Compare0) {
            // the compare register has already triggered
            TIMER_FLAG.store(false, Ordering::SeqCst);
            true
        } else {
            false
        }
    }) {
        return;
    }

    while !TIMER_FLAG.load(Ordering::SeqCst) {
        cortex_m::asm::wfi();
    }
    TIMER_FLAG.store(false, Ordering::SeqCst);
}

/// Set this timer to interrupt at the given frequency.
/// The next interrupt will be after 1/hz seconds.
///
#[allow(clippy::missing_panics_doc)]
pub fn set_tick_frequency(hz: u64) {
    RTC.modify(|rtc| {
        let counter_setting = (1_000_000_000 / hz) / PERIOD;
        // this can't actually happen, since it'd need hz < 1
        debug_assert!(counter_setting < (1 << 24), "counter period should be less than 1<<24 (roughly 6 minutes with the default PRESCALER settings)");

        COUNTER_PERIOD.store(counter_setting as u32, Ordering::SeqCst);

        rtc.set_compare(RtcCompareReg::Compare0, counter_setting as u32)
            .unwrap();
        rtc.clear_counter();
        PREV_COUNTER.store(0, Ordering::SeqCst);
    });

    // Give the counter time to clear (Section 19.1.8 of the NRF51 reference manual V3)
    delay_us_assembly(100);
}

/// Delay the program for a time using assembly instructions.
/// Testing shows this overshoots by ~5%, which is the closest that is possible without undershooting.
#[allow(unused_assignments)]
pub fn delay_us_assembly(mut number_of_us: u32) {
    unsafe {
        asm!(
        "1:",
        "subs {}, #1",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "nop",
        "bne 1b",
        inout(reg) number_of_us,
        options(nomem, nostack)
        );
    }
}

/// Delay the program for a time using assembly instructions.
/// Testing shows this overshoots by ~5%, which is the closest that is possible without undershooting.
pub fn delay_ms_assembly(number_of_ms: u32) {
    for _ in 0..number_of_ms {
        delay_us_assembly(999);
    }
}
