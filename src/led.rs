//! User LEDs

use stm32f40x::{GPIOA, RCC};

/// All the user LEDs
pub static LEDS: [Led; 1] = [
    Led { i: 5 },
];

/// An LED
pub struct Led {
    i: u8,
}

impl Led {
    /// Turns off the LED
    pub fn off(&self) {
        // NOTE(safe) atomic write
        unsafe { (*GPIOA.get()).bsrr.write(|w| w.bits(1 << (self.i + 16))) }
    }

    /// Turns on the LED
    pub fn on(&self) {
        // NOTE(safe) atomic write
        unsafe { (*GPIOA.get()).bsrr.write(|w| w.bits(1 << self.i)) }
    }
}

/// Initializes all the user LEDs
pub fn init(gpioa: &GPIOA, rcc: &RCC) {
    // Power up peripherals
    rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit());

    // Configure pins 8-15 as outputs
    gpioa
        .moder
        .modify(
            |_, w| {
                w.moder5()
                    .bits(1)
            },
        );
}
