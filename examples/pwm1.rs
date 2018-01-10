//! PWM test
#![deny(unsafe_code)]
#![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m_rtfm as rtfm;
extern crate f4;

use f4::prelude::*;
use f4::time::Hertz;
use f4::{Channel, Pwm};
use rtfm::app;

const FREQUENCY: Hertz = Hertz(1000);

app! {
    device: f4::stm32f40x,
}

fn init(p: init::Peripherals) {
    let pwm = Pwm(p.TIM2);

    const CHANNELS: [Channel; 3] = [Channel::_1, Channel::_2, Channel::_3];

    for c in &CHANNELS {
        pwm.init(
            FREQUENCY.invert(),
            *c,
            None,
            p.GPIOA,
            p.GPIOB,
            p.GPIOC,
            p.RCC,
        );
        pwm.set_duty(*c, pwm.get_max_duty() / 16);
        pwm.enable(*c);
    }
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}
