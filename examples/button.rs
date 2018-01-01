//! Toggle LED by pressing the blue user button
// #![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate f4;

use f4::led::{self, LED};
use rtfm::{app, Threshold};

// TASKS & RESOURCES
app! {
    device: f4::stm32f40x,

    resources: {
        static ON: bool = false;
    },

    tasks: {
        EXTI15_10: {
            path: button,
            priority: 1,
            resources: [EXTI, ON],
        },
    },
}

// TASKS
// Toggle the state of the LED
fn button(_t: &mut Threshold, r: EXTI15_10::Resources) {
    **r.ON = !**r.ON;
    if **r.ON {
        LED.on();
    } else {
        LED.off();
    }
    // 10.3.6 Pending register
    r.EXTI.pr.modify(|_, w| w.pr13().set_bit());
}

// INITIALIZATION PHASE
fn init(p: init::Peripherals, _r: init::Resources) {
    led::init(p.GPIOA, p.RCC);

    // Enable GPIOC
    p.RCC.ahb1enr.modify(|_, w| w.gpiocen().set_bit());
    // Configure PC13 as input with pull-downs, RM0368 Table 23
    p.GPIOC.moder.modify(|_, w| unsafe { w.moder13().bits(0) });
    p.GPIOC
        .pupdr
        .modify(|_, w| unsafe { w.pupdr13().bits(0b10) });
    // System configuration controller clock enable
    p.RCC.apb2enr.modify(|_, w| w.syscfgen().set_bit());
    // Enable external interrupt RM0368 7.2.6
    p.SYSCFG
        .exticr4
        .modify(|_, w| unsafe { w.exti13().bits(0b0010) });
    // Interrupt request from line 13 is not masked
    p.EXTI.imr.modify(|_, w| w.mr13().set_bit());
    // Rising edge trigger
    p.EXTI.rtsr.modify(|_, w| w.tr13().set_bit());
    // Falling edge trigger
    // p.EXTI.ftsr.modify(|_, w| w.tr13().set_bit());
}

// IDLE LOOP
fn idle() -> ! {
    // Sleep
    loop {
        rtfm::wfi();
    }
}
