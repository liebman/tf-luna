//! Register definitions and bitfield structures for the TF-Luna lidar.
//!
//! This module contains the register address map, bitfield definitions, and
//! related types used to interact with the TF-Luna over I²C.
//!
//! All multi-byte measurement registers are little-endian.

use bitfield::bitfield;

/// I²C address of the TF-Luna.
///
/// The device is factory programmed to this address; it is dynamic and may be
/// changed at runtime with `set_slave_address` (takes effect after a reboot).
pub const ADDRESS: u8 = 0x10;

/// Register addresses for the TF-Luna.
#[allow(unused)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RegAddr {
    /// Distance to target, low byte (cm)
    DistLow = 0x00,
    /// Distance to target, high byte (cm)
    DistHigh = 0x01,
    /// Signal amplitude, low byte
    AmpLow = 0x02,
    /// Signal amplitude, high byte
    AmpHigh = 0x03,
    /// Chip temperature, low byte
    TempLow = 0x04,
    /// Chip temperature, high byte
    TempHigh = 0x05,
    /// Tick counter, low byte
    TickLow = 0x06,
    /// Tick counter, high byte
    TickHigh = 0x07,
    /// Error status, low byte
    ErrorLow = 0x08,
    /// Error status, high byte
    ErrorHigh = 0x09,
    /// Version revision
    VersionRevision = 0x0a,
    /// Version minor
    VersionMinor = 0x0b,
    /// Version major
    VersionMajor = 0x0c,
    /// Serial number (14 bytes ASCII, first byte is 0x10)
    SerialNumber = 0x10,
    /// Ultra-low-power mode
    UltraLowPower = 0x1f,
    /// Save current settings to flash
    Save = 0x20,
    /// Reboot the device
    Reboot = 0x21,
    /// Slave (I²C) address
    SlaveAddr = 0x22,
    /// Operating mode (continuous / trigger)
    Mode = 0x23,
    /// Trigger a single measurement (trigger mode only)
    TrigOneShot = 0x24,
    /// Data output enable
    Enable = 0x25,
    /// Frame rate (FPS), low byte
    FpsLow = 0x26,
    /// Frame rate (FPS), high byte
    FpsHigh = 0x27,
    /// Low power mode
    LowPower = 0x28,
    /// Restore factory settings
    RestoreFactory = 0x29,
    /// Amplitude threshold, low byte
    AmpThrLow = 0x2a,
    /// Amplitude threshold, high byte
    AmpThrHigh = 0x2b,
    /// Dummy distance, low byte (cm)
    DummyDistLow = 0x2c,
    /// Dummy distance, high byte (cm)
    DummyDistHigh = 0x2d,
    /// Minimum measurable distance, low byte (cm)
    MinDistLow = 0x2e,
    /// Minimum measurable distance, high byte (cm)
    MinDistHigh = 0x2f,
    /// Maximum measurable distance, low byte (cm)
    MaxDistLow = 0x30,
    /// Maximum measurable distance, high byte (cm)
    MaxDistHigh = 0x31,
    /// Device signature (4 bytes ASCII: `LUNA`)
    Signature = 0x3c,
}

/// Operating mode of the TF-Luna.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Mode {
    /// Continuous measurement at the configured frame rate.
    Continuous = 0x00,
    /// Measurement is triggered on demand via [`RegAddr::TrigOneShot`].
    Trigger = 0x01,
}

impl From<Mode> for u8 {
    fn from(v: Mode) -> Self {
        v as u8
    }
}

impl TryFrom<u8> for Mode {
    type Error = u8;

    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0x00 => Ok(Mode::Continuous),
            0x01 => Ok(Mode::Trigger),
            other => Err(other),
        }
    }
}

/// Value written to [`RegAddr::Save`] to persist settings to flash.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Save {
    /// Commit the current settings to non-volatile storage.
    Save = 0x01,
}

/// Value written to [`RegAddr::Reboot`] to reboot the device.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Reboot {
    /// Reboot the device.
    Reboot = 0x02,
}

/// Value written to [`RegAddr::TrigOneShot`] to trigger a single measurement.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum Trigger {
    /// Trigger one measurement.
    Trigger = 0x01,
}

/// Value written to [`RegAddr::RestoreFactory`] to restore factory settings.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum RestoreFactory {
    /// Restore factory defaults.
    Restore = 0x01,
}

/// Value written to [`RegAddr::UltraLowPower`] to configure ultra-low-power mode.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum UltraLowPower {
    /// Disable ultra-low-power mode.
    Disabled = 0x00,
    /// Enable ultra-low-power mode.
    Enabled = 0x01,
}

bitfield! {
    /// Data output enable register.
    ///
    /// When disabled the device stops updating the measurement registers,
    /// which is useful to obtain a stable reading.
    #[derive(Clone, Copy, Default, PartialEq, Eq)]
    pub struct Enable(u8);
    impl Debug;
    /// `true` when data output is enabled (register value `0x00`).
    pub enabled, set_enabled: 0;
}

impl Enable {
    /// Creates an [`Enable`] from a raw register value.
    ///
    /// Register `0x00` means the output is enabled and any other value means
    /// it is disabled (only bit 0 is considered).
    #[must_use]
    pub fn from_register(value: u8) -> Self {
        let mut enable = Enable(0);
        enable.set_enabled(value & 0x01 == 0);
        enable
    }

    /// Returns the raw register value representing this state.
    #[must_use]
    pub fn to_register(self) -> u8 {
        u8::from(!self.enabled())
    }
}

bitfield! {
    /// Error status register (16-bit).
    ///
    /// See the TF-Luna datasheet for the meaning of the individual bits.
    #[derive(Clone, Copy, Default, PartialEq, Eq)]
    pub struct Error(u16);
    impl Debug;
    /// Signal amplitude is too low to produce a valid measurement.
    pub amp_low, set_amp_low: 0;
    /// Signal amplitude is too high (saturated).
    pub amp_high, set_amp_high: 1;
    /// Measured distance is below the configured minimum.
    pub dist_below_min, set_dist_below_min: 2;
    /// Measured distance is above the configured maximum.
    pub dist_above_max, set_dist_above_max: 3;
}

/// Creates the `From<u8>`/`From<Ty> for u8` conversions for a newtype bitfield.
macro_rules! from_register_u8 {
    ($typ:ty) => {
        impl From<u8> for $typ {
            fn from(v: u8) -> Self {
                paste::paste!([< $typ >](v))
            }
        }
        impl From<$typ> for u8 {
            fn from(v: $typ) -> Self {
                v.0
            }
        }
    };
}

from_register_u8!(Enable);

impl From<u16> for Error {
    fn from(v: u16) -> Self {
        Error(v)
    }
}

impl From<Error> for u16 {
    fn from(v: Error) -> Self {
        v.0
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for Error {
    fn format(&self, f: defmt::Formatter) {
        defmt::write!(
            f,
            "Error(amp_low={}, amp_high={}, dist_below_min={}, dist_above_max={})",
            self.amp_low(),
            self.amp_high(),
            self.dist_below_min(),
            self.dist_above_max()
        );
    }
}


