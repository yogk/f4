//! Toggle LED by interrupt when pressing the blue user button on PC13
//!
//! ```
//! // #![deny(warnings)]
//! #![feature(proc_macro)]
//! #![no_std]
//!
//! extern crate cortex_m;
//! extern crate cortex_m_rtfm as rtfm;
//! extern crate f4;
//!
//! use f4::led::{self, LED};
//! use f4::button::{self, BUTTON};
//! use rtfm::{app, Threshold};
//!
//! // TASKS & RESOURCES
//! app! {
//!     device: f4::stm32f40x,
//!
//!     resources: {
//!         static ON: bool = false;
//!     },
//!
//!     tasks: {
//!         EXTI15_10: {
//!             path: button,
//!             priority: 1,
//!             resources: [EXTI, ON],
//!         },
//!     },
//! }
//!
//! // TASKS
//! // Toggle the state of the LED
//! fn button(_t: &mut Threshold, r: EXTI15_10::Resources) {
//!     **r.ON = !**r.ON;
//!     if **r.ON {
//!         LED.on();
//!     } else {
//!         LED.off();
//!     }
//!     // Clear the button interrupt
//!     BUTTON.clear_pending(&r.EXTI);
//! }
//!
//! // INITIALIZATION PHASE
//! fn init(p: init::Peripherals, _r: init::Resources) {
//!     led::init(p.GPIOA, p.RCC);
//!     button::init(p.GPIOC, p.RCC, p.SYSCFG, p.EXTI);
//! }
//!
//! // IDLE LOOP
//! fn idle() -> ! {
//!     // Sleep
//!     loop {
//!         rtfm::wfi();
//!     }
//! }
//! ```
// Auto-generated. Do not modify.
