//! Board Support Crate for the STM32F3DISCOVERY
//!
//! # Usage
//!
//! Follow `cortex-m-quickstart` [instructions][i] but remove the `memory.x`
//! linker script and the `build.rs` build script file as part of the
//! configuration of the quickstart crate. Additionally, uncomment the "if using
//! ITM" block in the `.gdbinit` file.
//!
//! [i]: https://docs.rs/cortex-m-quickstart/0.2.0/cortex_m_quickstart/
//!
//! # Examples
//!
//! Check the [examples] module.
//!
//! [examples]: ./examples/index.html

#![deny(missing_docs)]
// #![deny(warnings)]
#![feature(const_fn)]
#![feature(const_unsafe_cell_new)]
#![feature(const_cell_new)]
#![feature(get_type_id)]
#![feature(never_type)]
#![feature(unsize)]
#![no_std]

extern crate cast;
extern crate embedded_hal as hal;
extern crate nb;
extern crate static_ref;

pub extern crate stm32f40x;

// For documentation only
pub mod examples;

pub mod dma;
pub mod led;
pub mod leds;
pub mod serial;
pub mod timer;
pub mod time;
pub mod pwm;
pub mod capture;
pub mod clock;
pub mod spi;

pub mod frequency;
use frequency::*;

pub use hal::prelude;
pub use serial::Serial;
pub use serial::U8Writer;
pub use timer::{Channel, Timer};
pub use pwm::Pwm;
pub use capture::Capture;
pub use spi::Spi;

/// println over semihosting
#[macro_export]
macro_rules! println {
    ($($e:tt)*) => {
        {
            extern crate cortex_m_semihosting;
            use core::fmt::Write;
            writeln!(cortex_m_semihosting::hio::hstdout().unwrap(), $($e)*).unwrap();
        }
    }
}
