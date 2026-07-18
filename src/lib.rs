/*!
 * This is a Rust embedded-hal implementation of the DecaWave DW1000.
 *
 * It requires an unstable toolchain for #[feature(inherent_associated_types)].
 *
 * DW3000 support is planned.
 */

#![no_std]
#![feature(inherent_associated_types)]
#![deny(unused_must_use)]
#![warn(missing_docs)]

pub mod registers;

pub mod dw1000;

// TODO: finish implementing dw3000
//pub mod dw3000;
