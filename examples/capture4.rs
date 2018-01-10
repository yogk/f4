//! Input capture using TIM4
#![deny(unsafe_code)]
// #![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

#[macro_use]
extern crate cortex_m;
#[macro_use]
extern crate cortex_m_debug;
extern crate cortex_m_rtfm as rtfm;
extern crate f4;
extern crate nb;

use f4::clock;
use f4::time::{Milliseconds, Hertz};
use f4::{Capture, Channel};
use f4::prelude::*;
use rtfm::{app, Threshold};

const RESOLUTION: Milliseconds = Milliseconds(1);
const CHANNELS: [Channel; 1] = [Channel::_1];

app! {
    device: f4::stm32f40x,

    idle: {
        resources: [TIM2],
    },
}

fn init(p: init::Peripherals) {
    clock::set_84_mhz(&p.RCC, &p.FLASH);
    let capture = Capture(p.TIM2);

    for c in &CHANNELS {
        capture.init(RESOLUTION, *c, p.GPIOA, p.GPIOB, p.GPIOC, p.RCC);

        capture.enable(*c);
    }
}

fn idle(_t: &mut Threshold, r: idle::Resources) -> ! {
    let mut t0: u32 = 0;
    let capture = Capture(&*r.TIM2);
    loop {
        for c in &CHANNELS {
            match capture.capture(*c) {
                Ok(t1) => {
                    // Calcualte delta time since last capture
                    let dt: u32 = t1.wrapping_sub(t0);
                    t0 = t1;
                    // Time is in APB1 peripheral clock domain
                    let dt_ticks = f4::frequency::apb1::Ticks(dt);
                    // Convert it to system time in milliseconds
                    let dt_ms: Milliseconds = f4::frequency::ahb1::Ticks(dt_ticks.into()).into();
                    // Print it to ITM
                    ipln!("{:?}: {:?} ms", c, dt_ms);
                }
                Err(nb::Error::WouldBlock) => {} // Keep looping
                Err(nb::Error::Other(f4::capture::Error::Overcapture)) => {
                    ipln!("{:?}: Overcapture", c);
                    capture.clear(*c);
                }
                Err(nb::Error::Other(e)) => {
                    ipln!("{:?}: {:?}", c, e);
                    rtfm::bkpt();
                }
            }
        }
    }
}
