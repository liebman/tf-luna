//! # TF-Luna lidar Driver
//!
//! A platform-agnostic Rust driver for the Benewake TF-Luna lidar range sensor,
//! built on the `embedded-hal` ecosystem. The crate is `no_std` and works
//! with any HAL that implements the `embedded-hal` (or `embedded-hal-async`)
//! I²C traits.
//!
//! ## Features
//!
//! - Both blocking and async I²C operation support
//! - Full register access (measurement, configuration, commands)
//! - Optional logging support via `log` or `defmt`
//! - No `unsafe` code
//!
//! ### Blocking Usage
//!
//! ```rust,ignore
//! use tf_luna::TfLuna;
//!
//! // Create the driver on the default I²C address (0x10)
//! let mut luna = TfLuna::new(i2c);
//!
//! // Verify the device is present and configured
//! luna.verify()?;
//!
//! // Read a full measurement
//! let sample = luna.read()?;
//! // `sample.distance` is in centimetres
//! ```
//!
//! ### Async Usage
//!
//! Enable the `async` feature in your `Cargo.toml` and use with `async`/`await`:
//!
//! ```rust,ignore
//! use tf_luna::TfLuna;
//!
//! let mut luna = TfLuna::new(i2c);
//! luna.verify().await?;
//! let sample = luna.read().await?;
//! ```
//!
//! ## Features
//!
//! - `async` - Enables optional async I²C support
//! - `log` - Enables logging via the `log` crate
//! - `defmt` - Enables logging via the `defmt` crate
//!
//! ## Register Map
//!
//! The driver provides access to the TF-Luna register map:
//!
//! - Measurement: distance, amplitude, temperature, tick, error
//! - Identity: signature, version, serial number
//! - Configuration: mode, frame rate, enable, amplitude threshold, distance limits
//! - Commands: save, reboot, one-shot trigger, restore factory, slave address
//!
//! See [`registers`] for the complete register address map.

#![no_std]
#![warn(missing_docs)]
#![warn(clippy::all)]
#![warn(clippy::pedantic)]
// MUST be the first module
mod fmt;

pub mod registers;

#[cfg(not(feature = "async"))]
use embedded_hal::i2c::I2c;
#[cfg(feature = "async")]
use embedded_hal_async::i2c::I2c;

pub use crate::registers::{
    Enable, Error, Mode, Reboot, RegAddr, RestoreFactory, Save, Trigger, UltraLowPower, ADDRESS,
};

/// Default I²C address of the TF-Luna.
pub const DEFAULT_ADDRESS: u8 = registers::ADDRESS;

/// Error type for TF-Luna operations.
///
/// The type is generic over the I²C bus error `I2CE` so the driver can be used
/// with any `embedded-hal` I²C implementation without leaking a concrete error
/// type.
#[derive(Debug, PartialEq, Eq)]
pub enum TfLunaError<I2CE> {
    /// I²C bus error
    I2c(I2CE),
    /// The device signature is not `LUNA`
    InvalidSignature,
    /// The device reported an unknown operating mode
    InvalidMode(u8),
    /// A distance value outside the sensor's valid range was provided
    InvalidDistance(u16),
    /// A frame rate outside the sensor's valid range was provided
    InvalidFps(u16),
    /// The requested operation returned an error status
    Device(Error),
}

impl<I2CE> From<I2CE> for TfLunaError<I2CE> {
    /// Creates a `TfLunaError` from an I²C error.
    fn from(e: I2CE) -> Self {
        TfLunaError::I2c(e)
    }
}

/// Device signature (`LUNA`).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Signature([u8; 4]);

impl Signature {
    /// Returns the raw signature bytes.
    #[must_use]
    pub fn bytes(&self) -> [u8; 4] {
        self.0
    }

    /// Returns `true` if the signature is the expected `LUNA`.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        &self.0 == b"LUNA"
    }
}

impl core::fmt::Display for Signature {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for b in self.0 {
            if b.is_ascii_graphic() {
                write!(f, "{}", b as char)?;
            } else {
                write!(f, ".")?;
            }
        }
        Ok(())
    }
}

/// Firmware version of the device.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Version {
    /// Major version
    pub major: u8,
    /// Minor version
    pub minor: u8,
    /// Revision
    pub revision: u8,
}

impl core::fmt::Display for Version {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}.{}.{}", self.major, self.minor, self.revision)
    }
}

/// A single measurement sample read from the device.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Measurement {
    /// Distance to the target in centimetres
    pub distance: u16,
    /// Signal amplitude
    pub amplitude: u16,
    /// Chip temperature in degrees Celsius
    pub temperature: f32,
    /// Tick counter
    pub tick: u16,
    /// Error status
    pub error: Error,
}

/// TF-Luna lidar range sensor driver.
///
/// The driver is generic over the I²C bus `I2C`. With the `async` feature
/// enabled, `I2C` must implement [`embedded_hal_async::i2c::I2c`]; otherwise it
/// must implement [`embedded_hal::i2c::I2c`].
pub struct TfLuna<I2C> {
    i2c: I2C,
    address: u8,
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), keep_self),
    async(feature = "async", keep_self)
)]
impl<I2C, E> TfLuna<I2C>
where
    I2C: I2c<Error = E>,
{
    /// Creates a new TF-Luna driver instance on the default address (0x10).
    pub fn new(i2c: I2C) -> Self {
        Self {
            i2c,
            address: DEFAULT_ADDRESS,
        }
    }

    /// Creates a new TF-Luna driver instance on a custom I²C address.
    ///
    /// Use this when the device address has been changed from the factory
    /// default with [`Self::set_slave_address`].
    pub fn with_address(i2c: I2C, address: u8) -> Self {
        Self { i2c, address }
    }

    /// Returns the I²C address the driver is currently using.
    #[must_use]
    pub fn address(&self) -> u8 {
        self.address
    }

    /// Consumes the driver, returning the underlying I²C bus.
    pub fn release(self) -> I2C {
        self.i2c
    }

    /// Verifies that the device is a TF-Luna by reading its signature.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::InvalidSignature`] if the device does not report
    /// the `LUNA` signature, or [`TfLunaError::I2c`] on a bus error.
    pub async fn verify(&mut self) -> Result<(), TfLunaError<E>> {
        let signature = self.signature().await?;
        debug!("TF-Luna signature: {}", signature);
        if signature.is_valid() {
            Ok(())
        } else {
            Err(TfLunaError::InvalidSignature)
        }
    }

    /// Verifies the device and applies an initial operating mode and frame rate.
    ///
    /// This is a convenience helper that calls [`Self::verify`], then
    /// [`Self::set_mode`] and [`Self::set_fps`].
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::InvalidSignature`] if the device does not report
    /// the `LUNA` signature, [`TfLunaError::InvalidFps`] if `fps` is outside the
    /// supported range, or [`TfLunaError::I2c`] on a bus error.
    pub async fn init(&mut self, mode: Mode, fps: u16) -> Result<(), TfLunaError<E>> {
        self.verify().await?;
        self.set_mode(mode).await?;
        self.set_fps(fps).await?;
        Ok(())
    }

    /// Reads the device signature (4 ASCII bytes).
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn signature(&mut self) -> Result<Signature, TfLunaError<E>> {
        let mut buf = [0u8; 4];
        self.read_bytes(RegAddr::Signature, &mut buf).await?;
        Ok(Signature(buf))
    }

    /// Reads the firmware version.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn version(&mut self) -> Result<Version, TfLunaError<E>> {
        let mut buf = [0u8; 3];
        self.read_bytes(RegAddr::VersionRevision, &mut buf).await?;
        Ok(Version {
            revision: buf[0],
            minor: buf[1],
            major: buf[2],
        })
    }

    /// Reads the 14-byte ASCII serial number.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn serial_number(&mut self) -> Result<[u8; 14], TfLunaError<E>> {
        let mut buf = [0u8; 14];
        self.read_bytes(RegAddr::SerialNumber, &mut buf).await?;
        Ok(buf)
    }

    /// Reads the distance to the target in centimetres.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn distance(&mut self) -> Result<u16, TfLunaError<E>> {
        self.read_u16(RegAddr::DistLow).await
    }

    /// Reads the signal amplitude.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn amplitude(&mut self) -> Result<u16, TfLunaError<E>> {
        self.read_u16(RegAddr::AmpLow).await
    }

    /// Reads the chip temperature in degrees Celsius.
    ///
    /// The raw register value is converted with `raw / 8 - 256`.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn temperature(&mut self) -> Result<f32, TfLunaError<E>> {
        let raw = self.read_u16(RegAddr::TempLow).await?;
        Ok(f32::from(raw) / 8.0 - 256.0)
    }

    /// Reads the raw (unscaled) temperature register value.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn temperature_raw(&mut self) -> Result<u16, TfLunaError<E>> {
        self.read_u16(RegAddr::TempLow).await
    }

    /// Reads the tick counter.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn tick(&mut self) -> Result<u16, TfLunaError<E>> {
        self.read_u16(RegAddr::TickLow).await
    }

    /// Reads and decodes the 16-bit error status register.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn error_status(&mut self) -> Result<Error, TfLunaError<E>> {
        let raw = self.read_u16(RegAddr::ErrorLow).await?;
        Ok(raw.into())
    }

    /// Reads a complete measurement sample.
    ///
    /// This performs five reads (distance, amplitude, temperature, tick and
    /// error) and assembles them into a [`Measurement`].
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn read(&mut self) -> Result<Measurement, TfLunaError<E>> {
        let distance = self.distance().await?;
        let amplitude = self.amplitude().await?;
        let temperature = self.temperature().await?;
        let tick = self.tick().await?;
        let error = self.error_status().await?;
        Ok(Measurement {
            distance,
            amplitude,
            temperature,
            tick,
            error,
        })
    }

    /// Reads the operating mode.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error, or
    /// [`TfLunaError::InvalidMode`] if the device reports an unknown mode.
    pub async fn mode(&mut self) -> Result<Mode, TfLunaError<E>> {
        let raw = self.read_byte(RegAddr::Mode).await?;
        Mode::try_from(raw).map_err(TfLunaError::InvalidMode)
    }

    /// Sets the operating mode.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn set_mode(&mut self, mode: Mode) -> Result<(), TfLunaError<E>> {
        debug!("TF-Luna set_mode: {:?}", mode);
        self.write_byte(RegAddr::Mode, mode as u8).await
    }

    /// Reads the frame rate in frames per second (Hz).
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn fps(&mut self) -> Result<u16, TfLunaError<E>> {
        self.read_u16(RegAddr::FpsLow).await
    }

    /// Sets the frame rate in frames per second (Hz).
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::InvalidFps`] if `fps` is outside the supported
    /// `1..=1000` range.
    pub async fn set_fps(&mut self, fps: u16) -> Result<(), TfLunaError<E>> {
        if !(1..=1000).contains(&fps) {
            return Err(TfLunaError::InvalidFps(fps));
        }
        debug!("TF-Luna set_fps: {}", fps);
        self.write_u16(RegAddr::FpsLow, fps).await
    }

    /// Reads the amplitude threshold.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn amp_threshold(&mut self) -> Result<u16, TfLunaError<E>> {
        self.read_u16(RegAddr::AmpThrLow).await
    }

    /// Sets the amplitude threshold.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn set_amp_threshold(&mut self, threshold: u16) -> Result<(), TfLunaError<E>> {
        debug!("TF-Luna set_amp_threshold: {}", threshold);
        self.write_u16(RegAddr::AmpThrLow, threshold).await
    }

    /// Reads the dummy distance in centimetres.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn dummy_distance(&mut self) -> Result<u16, TfLunaError<E>> {
        self.read_u16(RegAddr::DummyDistLow).await
    }

    /// Sets the dummy distance in centimetres.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn set_dummy_distance(&mut self, distance: u16) -> Result<(), TfLunaError<E>> {
        self.write_u16(RegAddr::DummyDistLow, distance).await
    }

    /// Reads the minimum measurable distance in centimetres.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn min_distance(&mut self) -> Result<u16, TfLunaError<E>> {
        self.read_u16(RegAddr::MinDistLow).await
    }

    /// Sets the minimum measurable distance in centimetres.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn set_min_distance(&mut self, distance: u16) -> Result<(), TfLunaError<E>> {
        self.write_u16(RegAddr::MinDistLow, distance).await
    }

    /// Reads the maximum measurable distance in centimetres.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn max_distance(&mut self) -> Result<u16, TfLunaError<E>> {
        self.read_u16(RegAddr::MaxDistLow).await
    }

    /// Sets the maximum measurable distance in centimetres.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn set_max_distance(&mut self, distance: u16) -> Result<(), TfLunaError<E>> {
        self.write_u16(RegAddr::MaxDistLow, distance).await
    }

    /// Reads the low power mode register.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn low_power(&mut self) -> Result<u8, TfLunaError<E>> {
        self.read_byte(RegAddr::LowPower).await
    }

    /// Sets the low power mode register.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn set_low_power(&mut self, value: u8) -> Result<(), TfLunaError<E>> {
        self.write_byte(RegAddr::LowPower, value).await
    }

    /// Enables or disables ultra-low-power mode.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn set_ultra_low_power(
        &mut self,
        mode: UltraLowPower,
    ) -> Result<(), TfLunaError<E>> {
        self.write_byte(RegAddr::UltraLowPower, mode as u8).await
    }

    /// Reads the data output enable state.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn enabled(&mut self) -> Result<Enable, TfLunaError<E>> {
        let raw = self.read_byte(RegAddr::Enable).await?;
        Ok(Enable::from_register(raw))
    }

    /// Enables or disables the data output stream.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn set_enabled(&mut self, enable: Enable) -> Result<(), TfLunaError<E>> {
        self.write_byte(RegAddr::Enable, enable.to_register()).await
    }

    /// Triggers a single measurement (only valid in [`Mode::Trigger`]).
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn trigger_oneshot(&mut self) -> Result<(), TfLunaError<E>> {
        self.write_byte(RegAddr::TrigOneShot, Trigger::Trigger as u8)
            .await
    }

    /// Saves the current settings to non-volatile storage.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn save(&mut self) -> Result<(), TfLunaError<E>> {
        self.write_byte(RegAddr::Save, Save::Save as u8).await
    }

    /// Restores the factory settings.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn restore_factory(&mut self) -> Result<(), TfLunaError<E>> {
        self.write_byte(RegAddr::RestoreFactory, RestoreFactory::Restore as u8)
            .await
    }

    /// Reboots the device.
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn reboot(&mut self) -> Result<(), TfLunaError<E>> {
        self.write_byte(RegAddr::Reboot, Reboot::Reboot as u8).await
    }

    /// Sets the I²C slave address.
    ///
    /// The new address takes effect after a reboot (see [`Self::reboot`]). After
    /// changing the address, create a new driver with [`Self::with_address`].
    ///
    /// # Errors
    ///
    /// Returns [`TfLunaError::I2c`] if there is an I²C bus error.
    pub async fn set_slave_address(&mut self, address: u8) -> Result<(), TfLunaError<E>> {
        debug!("TF-Luna set_slave_address: 0x{:02x}", address);
        self.write_byte(RegAddr::SlaveAddr, address).await
    }

    async fn read_byte(&mut self, reg: RegAddr) -> Result<u8, TfLunaError<E>> {
        let mut buf = [0u8; 1];
        self.read_bytes(reg, &mut buf).await?;
        Ok(buf[0])
    }

    async fn read_u16(&mut self, reg: RegAddr) -> Result<u16, TfLunaError<E>> {
        let mut buf = [0u8; 2];
        self.read_bytes(reg, &mut buf).await?;
        Ok(u16::from_le_bytes(buf))
    }

    async fn read_bytes(&mut self, reg: RegAddr, buf: &mut [u8]) -> Result<(), TfLunaError<E>> {
        self.i2c
            .write_read(self.address, &[reg as u8], buf)
            .await
            .map_err(TfLunaError::I2c)
    }

    async fn write_byte(&mut self, reg: RegAddr, value: u8) -> Result<(), TfLunaError<E>> {
        self.i2c
            .write(self.address, &[reg as u8, value])
            .await
            .map_err(TfLunaError::I2c)
    }

    async fn write_u16(&mut self, reg: RegAddr, value: u16) -> Result<(), TfLunaError<E>> {
        let [lo, hi] = value.to_le_bytes();
        self.i2c
            .write(self.address, &[reg as u8, lo, hi])
            .await
            .map_err(TfLunaError::I2c)
    }
}

#[maybe_async_cfg::maybe(
    sync(cfg(not(feature = "async")), keep_self),
    async(feature = "async", keep_self)
)]
#[cfg(test)]
mod tests {
    extern crate alloc;
    use alloc::vec;

    use super::*;
    use embedded_hal::i2c::ErrorKind as I2cErrorKind;
    use embedded_hal_mock::eh1::i2c::{Mock as I2cMock, Transaction as I2cTrans};

    const DEVICE_ADDRESS: u8 = DEFAULT_ADDRESS;

    fn setup_mock(expectations: &[I2cTrans]) -> I2cMock {
        I2cMock::new(expectations)
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_signature_valid() {
        let mock = setup_mock(&[
            I2cTrans::write_read(
                DEVICE_ADDRESS,
                vec![RegAddr::Signature as u8],
                vec![b'L', b'U', b'N', b'A'],
            ),
            I2cTrans::write_read(
                DEVICE_ADDRESS,
                vec![RegAddr::Signature as u8],
                vec![b'L', b'U', b'N', b'A'],
            ),
        ]);
        let mut dev = TfLuna::new(mock);

        let sig = dev.signature().await.unwrap();
        assert!(sig.is_valid());
        assert_eq!(sig.bytes(), *b"LUNA");
        dev.verify().await.unwrap();

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_signature_invalid() {
        let mock = setup_mock(&[I2cTrans::write_read(
            DEVICE_ADDRESS,
            vec![RegAddr::Signature as u8],
            vec![b'X', b'X', b'X', b'X'],
        )]);
        let mut dev = TfLuna::new(mock);

        let err = dev.verify().await.unwrap_err();
        assert_eq!(err, TfLunaError::InvalidSignature);

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_init() {
        let mock = setup_mock(&[
            I2cTrans::write_read(
                DEVICE_ADDRESS,
                vec![RegAddr::Signature as u8],
                vec![b'L', b'U', b'N', b'A'],
            ),
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::Mode as u8, 0x00]),
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::FpsLow as u8, 0x64, 0x00]),
        ]);
        let mut dev = TfLuna::new(mock);

        dev.init(Mode::Continuous, 100).await.unwrap();

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_version() {
        let mock = setup_mock(&[I2cTrans::write_read(
            DEVICE_ADDRESS,
            vec![RegAddr::VersionRevision as u8],
            vec![0x01, 0x02, 0x03],
        )]);
        let mut dev = TfLuna::new(mock);

        let version = dev.version().await.unwrap();
        assert_eq!(
            version,
            Version {
                major: 3,
                minor: 2,
                revision: 1
            }
        );

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_serial_number() {
        let mock = setup_mock(&[I2cTrans::write_read(
            DEVICE_ADDRESS,
            vec![RegAddr::SerialNumber as u8],
            b"ABCDEFGHIJKLMN".to_vec(),
        )]);
        let mut dev = TfLuna::new(mock);

        let serial = dev.serial_number().await.unwrap();
        assert_eq!(&serial, b"ABCDEFGHIJKLMN");

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_distance_and_amplitude() {
        let mock = setup_mock(&[
            I2cTrans::write_read(
                DEVICE_ADDRESS,
                vec![RegAddr::DistLow as u8],
                vec![0x34, 0x12], // 0x1234 = 4660, little-endian
            ),
            I2cTrans::write_read(
                DEVICE_ADDRESS,
                vec![RegAddr::AmpLow as u8],
                vec![0xab, 0x00], // 0x00ab = 171
            ),
        ]);
        let mut dev = TfLuna::new(mock);

        assert_eq!(dev.distance().await.unwrap(), 4660);
        assert_eq!(dev.amplitude().await.unwrap(), 171);

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_temperature() {
        // raw 0x08c8 = 2248 -> 2248 / 8 - 256 = 25.0 degC
        let mock = setup_mock(&[
            I2cTrans::write_read(DEVICE_ADDRESS, vec![RegAddr::TempLow as u8], vec![0xc8, 0x08]),
            I2cTrans::write_read(DEVICE_ADDRESS, vec![RegAddr::TempLow as u8], vec![0xc8, 0x08]),
        ]);
        let mut dev = TfLuna::new(mock);

        assert_eq!(dev.temperature_raw().await.unwrap(), 2248);
        assert!((dev.temperature().await.unwrap() - 25.0).abs() < f32::EPSILON);

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_read_full_measurement() {
        let mock = setup_mock(&[
            I2cTrans::write_read(DEVICE_ADDRESS, vec![RegAddr::DistLow as u8], vec![0x64, 0x00]),
            I2cTrans::write_read(DEVICE_ADDRESS, vec![RegAddr::AmpLow as u8], vec![0x2a, 0x01]),
            I2cTrans::write_read(DEVICE_ADDRESS, vec![RegAddr::TempLow as u8], vec![0xc8, 0x08]),
            I2cTrans::write_read(DEVICE_ADDRESS, vec![RegAddr::TickLow as u8], vec![0x05, 0x00]),
            I2cTrans::write_read(DEVICE_ADDRESS, vec![RegAddr::ErrorLow as u8], vec![0x02, 0x00]),
        ]);
        let mut dev = TfLuna::new(mock);

        let m = dev.read().await.unwrap();
        assert_eq!(m.distance, 100);
        assert_eq!(m.amplitude, 298);
        assert!((m.temperature - 25.0).abs() < f32::EPSILON);
        assert_eq!(m.tick, 5);
        assert!(m.error.amp_high());
        assert!(!m.error.amp_low());

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_mode() {
        let mock = setup_mock(&[
            I2cTrans::write_read(DEVICE_ADDRESS, vec![RegAddr::Mode as u8], vec![0x01]),
            I2cTrans::write_read(DEVICE_ADDRESS, vec![RegAddr::Mode as u8], vec![0x00]),
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::Mode as u8, 0x01]),
        ]);
        let mut dev = TfLuna::new(mock);

        assert_eq!(dev.mode().await.unwrap(), Mode::Trigger);
        assert_eq!(dev.mode().await.unwrap(), Mode::Continuous);
        dev.set_mode(Mode::Trigger).await.unwrap();

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_invalid_mode() {
        let mock = setup_mock(&[I2cTrans::write_read(
            DEVICE_ADDRESS,
            vec![RegAddr::Mode as u8],
            vec![0x42],
        )]);
        let mut dev = TfLuna::new(mock);

        let err = dev.mode().await.unwrap_err();
        assert_eq!(err, TfLunaError::InvalidMode(0x42));

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_fps() {
        let mock = setup_mock(&[
            I2cTrans::write_read(DEVICE_ADDRESS, vec![RegAddr::FpsLow as u8], vec![0x64, 0x00]),
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::FpsLow as u8, 0xe8, 0x03]),
        ]);
        let mut dev = TfLuna::new(mock);

        assert_eq!(dev.fps().await.unwrap(), 100);
        // 1000 = 0x03e8, little-endian
        dev.set_fps(1000).await.unwrap();

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_fps_out_of_range() {
        let mock = setup_mock(&[]);
        let mut dev = TfLuna::new(mock);

        assert_eq!(dev.set_fps(0).await.unwrap_err(), TfLunaError::InvalidFps(0));
        assert_eq!(
            dev.set_fps(1001).await.unwrap_err(),
            TfLunaError::InvalidFps(1001)
        );

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_enable() {
        let mock = setup_mock(&[
            I2cTrans::write_read(DEVICE_ADDRESS, vec![RegAddr::Enable as u8], vec![0x00]),
            I2cTrans::write_read(DEVICE_ADDRESS, vec![RegAddr::Enable as u8], vec![0x01]),
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::Enable as u8, 0x01]),
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::Enable as u8, 0x00]),
        ]);
        let mut dev = TfLuna::new(mock);

        let mut enable = dev.enabled().await.unwrap();
        assert!(enable.enabled());
        enable = dev.enabled().await.unwrap();
        assert!(!enable.enabled());

        enable.set_enabled(false);
        dev.set_enabled(enable).await.unwrap();
        enable.set_enabled(true);
        dev.set_enabled(enable).await.unwrap();

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_config_registers() {
        let mock = setup_mock(&[
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::AmpThrLow as u8, 0x64, 0x00]),
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::DummyDistLow as u8, 0x00, 0x00]),
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::MinDistLow as u8, 0x14, 0x00]),
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::MaxDistLow as u8, 0x40, 0x06]),
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::LowPower as u8, 0x01]),
        ]);
        let mut dev = TfLuna::new(mock);

        dev.set_amp_threshold(100).await.unwrap();
        dev.set_dummy_distance(0).await.unwrap();
        dev.set_min_distance(20).await.unwrap();
        dev.set_max_distance(1600).await.unwrap();
        dev.set_low_power(1).await.unwrap();

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_commands() {
        let mock = setup_mock(&[
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::TrigOneShot as u8, 0x01]),
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::Save as u8, 0x01]),
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::Reboot as u8, 0x02]),
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::RestoreFactory as u8, 0x01]),
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::SlaveAddr as u8, 0x11]),
            I2cTrans::write(DEVICE_ADDRESS, vec![RegAddr::UltraLowPower as u8, 0x01]),
        ]);
        let mut dev = TfLuna::new(mock);

        dev.trigger_oneshot().await.unwrap();
        dev.save().await.unwrap();
        dev.reboot().await.unwrap();
        dev.restore_factory().await.unwrap();
        dev.set_slave_address(0x11).await.unwrap();
        dev.set_ultra_low_power(UltraLowPower::Enabled).await.unwrap();

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_i2c_error_propagation() {
        let mock = setup_mock(&[I2cTrans::write_read(
            DEVICE_ADDRESS,
            vec![RegAddr::DistLow as u8],
            vec![0x00, 0x00],
        )
        .with_error(I2cErrorKind::Other)]);
        let mut dev = TfLuna::new(mock);

        let err = dev.distance().await.unwrap_err();
        assert!(matches!(err, TfLunaError::I2c(I2cErrorKind::Other)));

        dev.release().done();
    }

    #[cfg_attr(feature = "async", tokio::test)]
    #[cfg_attr(not(feature = "async"), test)]
    async fn test_custom_address() {
        let mock = setup_mock(&[I2cTrans::write_read(
            0x22,
            vec![RegAddr::DistLow as u8],
            vec![0x01, 0x00],
        )]);
        let mut dev = TfLuna::with_address(mock, 0x22);

        assert_eq!(dev.address(), 0x22);
        assert_eq!(dev.distance().await.unwrap(), 1);

        dev.release().done();
    }
}

