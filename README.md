# TF-Luna LiDAR Range Sensor Driver

[![Crates.io](https://img.shields.io/crates/v/tf-luna.svg)](https://crates.io/crates/tf-luna)
[![Documentation](https://docs.rs/tf-luna/badge.svg)](https://docs.rs/tf-luna)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](README.md)

A platform-agnostic Rust driver for the Benewake TF-Luna LiDAR range sensor,
built on the `embedded-hal` ecosystem. The crate is `no_std` and works with any
HAL that implements the `embedded-hal` (or `embedded-hal-async`) I²C traits.

- Both blocking and async I²C operation support
- Full register access (measurement, configuration, commands)
- Optional logging support via `log` or `defmt`
- No `unsafe` code

## Usage

Add this to your `Cargo.toml`:

```toml
[dependencies]
tf-luna = "0.1.0"
```

### Blocking Example

```rust
use tf_luna::TfLuna;

// Create the driver on the default I²C address (0x10)
let mut luna = TfLuna::new(i2c);

// Verify the device is present
luna.verify()?;

// Read a full measurement
let sample = luna.read()?;
// `sample.distance` is in centimetres
```

### Async Example

Enable the `async` feature in your `Cargo.toml` and use with async/await:

```rust
use tf_luna::TfLuna;

let mut luna = TfLuna::new(i2c);

luna.verify().await?;
let sample = luna.read().await?;
```

## Features

The crate can be compiled with the following features:

- `async`: Enables async I²C support
- `log`: Enables logging via the `log` crate
- `defmt`: Enables logging via the `defmt` crate

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)
- MIT license ([LICENSE-MIT](LICENSE-MIT) or <http://opensource.org/licenses/MIT>)

at your option.
