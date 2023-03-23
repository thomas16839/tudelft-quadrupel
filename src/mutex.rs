use core::cell::UnsafeCell;

/// A mutual exclusion primitive useful for protecting shared data. It works by disabling interrupts while the lock is being held.
///
/// This is implementation is only sound on the NRF51822 or other single-core processors.
pub struct Mutex<T> {
    inner: UnsafeCell<T>,
}

// SAFETY: it is safe to share a Mutex between interrupts and
// other code. That's because there is only one thread, and the
// only way to access what is in a Mutex is by locking it, disabling
// interrupts. That means there are two cases:

// 1. We are in normal code, there can be no interrupt (since we turned those off)
// and since there's only one core, we know for sure we're alone in accessing the
// wrapped value
//
// 2. We are in an interrupt. In an interrupt, no other interrupts can occur. It
// is also impossible to have already locked the Mutex at this point, since to lock
// it outside an interrupt, interrupts had to be turned off. That means when we are
// in an interrupt, nothing else can have the Mutex locked, otherwise we could not
// actually be in an interrupt.
unsafe impl<T> Sync for Mutex<T> {}

impl<T> Mutex<T> {
    /// Create a new Mutex.
    pub const fn new(v: T) -> Self {
        Self {
            inner: UnsafeCell::new(v),
        }
    }

    /// Locks the mutex in a callback
    #[inline(always)]
    pub fn modify<U>(&self, f: impl FnOnce(&mut T) -> U) -> U {
        cortex_m::interrupt::free(|_| {
            // Safety: Interrupts are disabled, so the current code is the only one that can be running.
            // TODO make sure you can't lock the mutex in here
            f(unsafe { &mut *self.inner.get() })
        })
    }

    /// This function gets a reference to the inner `T` _without_ locking the lock.
    /// This is inherently unsafe.
    ///
    /// # Safety
    /// This function is only safe if you can guarantee that no mutable references to the contents of this lock exist.
    ///
    /// This generally can be used in the following cases:
    /// * You only access this mutex from within a single interrupt
    /// * You only access this mutex outside of interrupts, as interrupts don't break the mutex guarantees
    pub unsafe fn no_critical_section_lock(&self) -> &T {
        &*self.inner.get()
    }

    /// This function gets a mutable reference to the inner `T` _without_ locking the lock.
    /// This is inherently unsafe.
    ///
    /// # Safety
    /// This function is only safe if you can guarantee that no other references to the contents of this lock exist.
    ///
    /// This generally can be used in the following cases:
    /// * You only access this mutex from within a single interrupt
    /// * You only access this mutex outside of interrupts, as interrupts don't break the mutex guarantees
    #[allow(clippy::mut_from_ref)]
    pub unsafe fn no_critical_section_lock_mut(&self) -> &mut T {
        &mut *self.inner.get()
    }
}
