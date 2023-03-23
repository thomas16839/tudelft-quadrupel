use core::ops::{Deref, DerefMut};

/// A wrapper around a type, making sure that the contents
/// are only ever initialized *once*.
pub struct OnceCell<T> {
    v: Option<T>,
}

impl<T> OnceCell<T> {
    /// Create a new uninitialized [`OnceCell`], which will later be
    /// populated with a value
    #[must_use]
    pub const fn uninitialized() -> Self {
        Self { v: None }
    }

    /// Create a new initialized [`OnceCell`], which is already populated with a value.
    pub const fn new(v: T) -> Self {
        Self { v: Some(v) }
    }

    /// Initialize an empty [`OnceCell`] with a value.
    ///
    /// # Panics
    /// When the cell is already initialized
    pub fn initialize(&mut self, value: T) {
        assert!(self.v.is_none(), "already initialized");
        self.v = Some(value);
    }

    /// Check if the [`OnceCell`] is already initialized
    pub fn is_initialized(&self) -> bool {
        self.v.is_some()
    }

    /// Uninitialize the `once_cell`.
    /// NOTE: you probably don't want this
    ///
    /// # Safety
    ///
    /// Some code may depend on things being initialized.
    /// In general, don't use this. The template only uses is in situations which
    /// are already known not to be recoverable, ever, so uninitializing values
    /// is acceptable
    pub unsafe fn uninitialize(&mut self) {
        self.v = None;
    }
}

impl<T> Deref for OnceCell<T> {
    type Target = T;

    fn deref(&self) -> &Self::Target {
        self.v.as_ref().unwrap()
    }
}

impl<T> DerefMut for OnceCell<T> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        self.v.as_mut().unwrap()
    }
}
