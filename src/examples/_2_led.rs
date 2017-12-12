//! Turns all the user LEDs on
//!
//! ```
//! #![deny(unsafe_code)]
//! #![deny(warnings)]
//! #![feature(proc_macro)]
//! #![no_std]
//!
//! extern crate cortex_m_rtfm as rtfm;
//! extern crate f4;
//!
//! use f4::led::{self, LEDS};
//! use rtfm::app;
//!
//! // TASKS & RESOURCES
//! app! {
//!     device: f4::stm32f40x,
//! }
//!
//! // INITIALIZATION PHASE
//! fn init(p: init::Peripherals) {
//!     led::init(&p.GPIOA, &p.RCC);
//! }
//!
//! // IDLE LOOP
//! fn idle() -> ! {
//!     for led in &LEDS {
//!         led.on();
//!     }
//!
//!     // Sleep
//!     loop {
//!         rtfm::wfi();
//!     }
//! }
//! ```
// Auto-generated. Do not modify.
