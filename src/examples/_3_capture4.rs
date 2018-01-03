//! Input capture using TIM4
//!
//! ```
//! #![deny(unsafe_code)]
//! #![deny(warnings)]
//! #![feature(proc_macro)]
//! #![no_std]
//!
//! #[macro_use]
//! extern crate cortex_m;
//! extern crate cortex_m_rtfm as rtfm;
//! extern crate f4;
//! extern crate nb;
//!
//! use f4::time::Milliseconds;
//! use f4::{Capture, Channel};
//! use f4::prelude::*;
//! use rtfm::{app, Threshold};
//!
//! const RESOLUTION: Milliseconds = Milliseconds(1);
//! const CHANNELS: [Channel; 2] = [Channel::_1, Channel::_2];
//!
//! app! {
//!     device: f4::stm32f40x,
//!
//!     idle: {
//!         resources: [ITM, TIM2],
//!     },
//! }
//!
//! fn init(p: init::Peripherals) {
//!     let capture = Capture(p.TIM2);
//!
//!     for c in &CHANNELS {
//!         capture.init(RESOLUTION, *c, p.GPIOA, p.GPIOB, p.GPIOC, p.RCC);
//!         capture.enable(*c);
//!     }
//! }
//!
//! fn idle(_t: &mut Threshold, r: idle::Resources) -> ! {
//!     let capture = Capture(&*r.TIM2);
//!
//!     loop {
//!         for c in &CHANNELS {
//!             match capture.capture(*c) {
//!                 Ok(snapshot) => {
//!                     iprintln!(&r.ITM.stim[0], "{:?}: {:?} ms", c, snapshot);
//!                 }
//!                 Err(nb::Error::WouldBlock) => {}
//!                 Err(nb::Error::Other(e)) => {
//!                     iprintln!(&r.ITM.stim[0], "{:?}: {:?}", c, e);
//!                     panic!("{:?}", e);
//!                 }
//!             }
//!         }
//!     }
//! }
//! ```
// Auto-generated. Do not modify.
