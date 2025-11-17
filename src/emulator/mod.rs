//! Emulator frontend modules

/// GTK4-based emulator frontend
#[cfg(feature = "gtk")]
pub mod gtk;

/// minifb-based emulator frontend
#[cfg(feature = "minifb")]
pub mod minifb;
