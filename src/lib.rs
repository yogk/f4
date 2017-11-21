#![feature(const_cell_new)]
#![feature(const_unsafe_cell_new)]
#![feature(const_fn)]
#![feature(never_type)]
#![feature(unsize)]
#![no_std]

extern crate cast;
extern crate embedded_hal as hal;
extern crate nb;
extern crate untagged_option;
pub extern crate stm32f30x;

mod bb;
pub mod dma;
pub mod gpio;
pub mod prelude;
pub mod serial;
pub mod time;
pub mod timer;

pub use serial::Serial;
pub use timer::Timer;

macro_rules! frequency {
    ($FREQUENCY:expr) => {
        use time::*;

        /// Frequency
        pub const FREQUENCY: u32 = $FREQUENCY;

        /// Unit of time
        #[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
        pub struct Ticks(pub u32);

        impl Ticks {
            /// Applies the function `f` to the inner value
            pub fn map<F>(self, f: F) -> Self
                where F: FnOnce(u32) -> u32,
            {
                Ticks(f(self.0))
            }
        }

        impl From<Ticks> for Microseconds {
            fn from(ticks: Ticks) -> Self {
                Microseconds(ticks.0 / (FREQUENCY / 1_000_000))
            }
        }

        impl From<Ticks> for Milliseconds {
            fn from(ticks: Ticks) -> Self {
                Milliseconds(ticks.0 / (FREQUENCY / 1_000))
            }
        }

        impl From<Ticks> for Seconds {
            fn from(ticks: Ticks) -> Self {
                Seconds(ticks.0 / FREQUENCY)
            }
        }

        impl From<Bps> for Ticks {
            fn from(bps: Bps) -> Ticks {
                Ticks(FREQUENCY / bps.0)
            }
        }

        impl From<Hertz> for Ticks {
            fn from(hertz: Hertz) -> Ticks {
                Ticks(FREQUENCY / hertz.0)
            }
        }

        impl From<Microseconds> for Ticks {
            fn from(us: Microseconds) -> Ticks {
                Ticks(us.0 * (FREQUENCY / 1_000_000))
            }
        }

        impl From<Milliseconds> for Ticks {
            fn from(ms: Milliseconds) -> Ticks {
                Ticks(ms.0 * (FREQUENCY / 1_000))
            }
        }

        impl From<Seconds> for Ticks {
            fn from(s: Seconds) -> Ticks {
                Ticks(s.0 * FREQUENCY)
            }
        }
    }
}

/// Advance High-performance Bus (AHB)
pub mod ahb {
    frequency!(8_000_000);
}

/// Advance Peripheral Bus 1 (APB1)
pub mod apb1 {
    frequency!(8_000_000);
}

/// Advance Peripheral Bus 2 (APB2)
pub mod apb2 {
    frequency!(8_000_000);
}
