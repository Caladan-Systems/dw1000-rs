# DW1000-rs

This is a Rust implementation of the DecaWave DW1000, targeting [embedded_hal](https://docs.rs/embedded-hal/latest/embedded_hal/) and [embedded_hal_async](https://docs.rs/embedded-hal-async/latest/embedded_hal_async/).

Most of it is blocking.

The actual reference manual can be found [here](https://fcc.report/FCC-ID/2AAXVTNTMOD1/2787937.pdf).

## Stability
Currently, dw1000-rs requires the nightly toolchain. Work is underway to bring it fully into stable rust.

## Support
dw1000-rs is verified working on custom internal esp32-s3 boards, and should work fine on other esp32 targets.
It does not depend on any esp32-specific features, but is not tested on e.g. mspm0 or nrf5x yet.
