//! Prints "Hello, World" in the OpenOCD console
//!
//! ```
//! #![deny(unsafe_code)]
//! #![deny(warnings)]
//! #![feature(proc_macro)]
//! #![no_std]
//!
//! extern crate cortex_m_rtfm as rtfm;
//! #[macro_use]
//! extern crate f4;
//!
//! use rtfm::app;
//!
//! app! {
//!     device: f4::stm32f40x,
//! }
//!
//! fn init(_p: init::Peripherals) {}
//!
//! fn idle() -> ! {
//!     let f = 1.0;
//!     println!("Hello, world! {}", f);
//!
//!     rtfm::bkpt();
//!     loop {
//!         rtfm::wfi();
//!     }
//! }
//! ```
// Auto-generated. Do not modify.
