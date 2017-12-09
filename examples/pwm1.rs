//! Output a PWM with a duty cycle of ~6% on all the channels of TIM1
// FIXME doesn't seem to work :-(

#![deny(unsafe_code)]
#![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

extern crate f4;
extern crate cortex_m_rtfm as rtfm;

use f4::prelude::*;
use f4::time::Hertz;
use f4::{Channel, Pwm};
use rtfm::app;

const FREQUENCY: Hertz = Hertz(10_000);

app! {
    device: f4::stm32f40x,
}

fn init(p: init::Peripherals) {
    let pwm = Pwm(p.TIM1);

    pwm.init(FREQUENCY.invert(), None, p.GPIOA,p.GPIOB,p.GPIOC, p.RCC);
    let duty = pwm.get_max_duty() / 16;

    const CHANNELS: [Channel; 4] = [Channel::_1, Channel::_2, Channel::_3, Channel::_4];

    for c in &CHANNELS {
        pwm.set_duty(*c, duty);
    }

    for c in &CHANNELS {
        pwm.enable(*c);
        rtfm::bkpt();
    }
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}
