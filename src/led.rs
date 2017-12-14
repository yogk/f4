//! User LED PA5

use stm32f40x::{GPIOA, RCC};

/// LED connected to pin PA5
pub const LED: PA5 = PA5;

/// Pin PA5. There's an LED connected to this pin
pub struct PA5;

/// Initializes the user LED
pub fn init(gpioa: &GPIOA, rcc: &RCC) {
    // power on GPIOA
    rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit());

    // configure PA5 as output
    gpioa.moder.modify(|_, w| w.moder5().bits(1));
}

impl PA5 {
    /// Turns the LED on
    pub fn on(&self) {
        unsafe {
            (*GPIOA.get()).odr.modify(|_, w| w.odr5().bit(true));
        }
    }

    /// Turns the LED off
    pub fn off(&self) {
        unsafe {
            (*GPIOA.get()).odr.modify(|_, w| w.odr5().bit(false));
        }
    }
}
