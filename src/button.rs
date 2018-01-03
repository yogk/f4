//! User button PC13

use stm32f40x::{EXTI, GPIOC, RCC, SYSCFG};

/// Button connected to pin PC13
pub const BUTTON: PC13 = PC13;

/// Pin PC13. There's a button connected to this pin
pub struct PC13;

/// Initializes the user button with interrupt EXTI15_10
pub fn init(gpioc: &GPIOC, rcc: &RCC, syscfg: &SYSCFG, exti: &EXTI) {
    // Enable GPIOC
    rcc.ahb1enr.modify(|_, w| w.gpiocen().set_bit());
    // Configure PC13 as input with pull-downs, RM0368 Table 23
    gpioc.moder.modify(|_, w| unsafe { w.moder13().bits(0) });
    gpioc.pupdr.modify(|_, w| unsafe { w.pupdr13().bits(0b10) });
    // System configuration controller clock enable
    rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());
    // Enable external interrupt RM0368 7.2.6
    syscfg
        .exticr4
        .modify(|_, w| unsafe { w.exti13().bits(0b0010) });
    // Interrupt request from line 13 is not masked
    exti.imr.modify(|_, w| w.mr13().set_bit());
    // Falling edge trigger
    exti.ftsr.modify(|_, w| w.tr13().set_bit());
}

impl PC13 {
    /// True if button is pressed, false otherwise.
    pub fn is_pressed(&self, gpioc: &GPIOC) -> bool {
        gpioc.odr.read().odr13().bit_is_clear()
    }

    /// Clear the pending external interrupt line used by the button, PR13
    pub fn clear_pending(&self, exti: &EXTI) {
        // RM0368 10.3.6 Pending register
        exti.pr.modify(|_, w| w.pr13().set_bit());
    }
}
