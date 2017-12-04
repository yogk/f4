//! User LEDs

use stm32f40x::{GPIOB, RCC};

/// All the user LEDs
pub static LEDS: [Led; 8] = [
    Led { i: 2 },
    Led { i: 1 },
    Led { i: 15 },
    Led { i: 14 },
    Led { i: 13 },
    Led { i: 5 },
    Led { i: 4 },
    Led { i: 10 },
];

/// An LED
pub struct Led {
    i: u8,
}

impl Led {
    /// Turns off the LED
    pub fn off(&self) {
        // NOTE(safe) atomic write
        unsafe { (*GPIOB.get()).bsrr.write(|w| w.bits(1 << (self.i + 16))) }
    }

    /// Turns on the LED
    pub fn on(&self) {
        // NOTE(safe) atomic write
        unsafe { (*GPIOB.get()).bsrr.write(|w| w.bits(1 << self.i)) }
    }
}

/// Initializes all the user LEDs
pub fn init(gpioa: &GPIOB, rcc: &RCC) {
    // Power up peripherals
    rcc.ahb1enr.modify(|_, w| w.gpioben().set_bit());

    // Configure pins 8-15 as outputs
    gpioa
        .moder
        .modify(
            |_, w| {unsafe {
                w.moder2().bits(1)
                .moder1().bits(1)
                .moder15().bits(1)
                .moder14().bits(1)
                .moder13().bits(1)
                .moder5().bits(1)
                .moder4().bits(1)
                .moder10().bits(1)
                }
            },
        );
}
