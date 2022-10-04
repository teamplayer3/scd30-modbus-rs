# SCD30 modbus driver

[![Build Status](https://github.com/teamplayer3/scd30-modbus-rs/workflows/Rust/badge.svg)](https://github.com/teamplayer3/scd30-modbus-rs/actions?query=workflow%3ARust)
[![License](https://img.shields.io/badge/license-MIT%2FApache--2.0-blue.svg)](https://github.com/teamplayer3/scd30-modbus-rs)
[![Crates.io](https://img.shields.io/crates/v/scd30-modbus-rs.svg)](https://crates.io/crates/scd30-modbus-rs)
[![Documentation](https://docs.rs/scd30-modbus-rs/badge.svg)](https://docs.rs/scd30-modbus-rs)

Implementation of an SCD30 sensor driver using the [modbus protocol](https://de.wikipedia.org/wiki/Modbus) written in rust. This driver is written async/await style. Currently the [smol](https://github.com/smol-rs/smol) runtime is supported.

For more information about the sensor [here](https://sensirion.com/de/produkte/katalog/SCD30/).

## Example

```rust
let serial = ...; // serial bus to use

let mut driver = Scd30::new(serial);

driver.set_measurement_interval(Duration::from_secs(15)).await.unwrap();
driver.start_measuring().await.unwrap();

let measurements = driver.read().await.unwrap().unwrap();
```