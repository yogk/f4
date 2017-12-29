//! Timer interrupt test. LED should flicker unevenly.

#![deny(unsafe_code)]
#![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

extern crate f4;
extern crate stm32f40x;

extern crate cortex_m_rtfm as rtfm;

extern crate nb;

use f4::Timer;
use f4::led::{self, LED};
use f4::prelude::*;
use f4::time::Hertz;
use f4::clock;
use rtfm::{app, Threshold};

const FREQUENCY1: Hertz = Hertz(10);
const FREQUENCY2: Hertz = Hertz(6);

app! {
    device: f4::stm32f40x,

    tasks: {
        TIM2: {
            path: tim2,
            resources: [TIM2],
        },
        TIM3: {
            path: tim3,
            resources: [TIM3],
        },
    },
}

fn init(p: init::Peripherals) {
    // Set system clock in order to test that it works
    clock::set_84_mhz(&p.RCC, &p.FLASH);

    led::init(p.GPIOA, p.RCC);

    let timer = Timer(&*p.TIM2);
    timer.init(FREQUENCY1.invert(), p.RCC);
    timer.resume();
    let timer = Timer(&*p.TIM3);
    timer.init(FREQUENCY2.invert(), p.RCC);
    timer.resume();
}

fn tim2(_t: &mut Threshold, r: TIM2::Resources) {
    // Clear the interrupt flag (RM0368, 13.4.5)
    r.TIM2.sr.modify(|_, w| w.uif().clear_bit());
    LED.off();
}

fn tim3(_t: &mut Threshold, r: TIM3::Resources) {
    r.TIM3.sr.modify(|_, w| w.uif().clear_bit());
    LED.on();
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}
