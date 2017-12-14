//! Drive a ring of 24 WS2812 LEDs
//!
//! To test this demo connect the data-in pin of the LED ring to pin PA0
#![deny(unsafe_code)]
#![deny(warnings)]
#![feature(const_fn)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m_rtfm as rtfm;
extern crate f4;
#[macro_use]
extern crate nb;

use f4::dma::{Buffer, Dma1Channel2};
use f4::prelude::*;
use f4::time::Hertz;
use f4::{Channel, Pwm};
use rtfm::{app, Static, Threshold};

// CONFIGURATION
const FREQUENCY: Hertz = Hertz(200_000);
const _0: u8 = 3;
const _1: u8 = 5;

app! {
    device: f4::stm32f40x,

    resources: {
        static BUFFER: Buffer<[u8; 577], Dma1Channel2> = Buffer::new([_0; 577]);
    },

    idle: {
        resources: [BUFFER, DMA1, TIM3],
    },
}

fn init(p: init::Peripherals, r: init::Resources) {
    let pwm = Pwm(p.TIM3);

    pwm.init(
        FREQUENCY.invert(),
        Channel::_1,
        Some(p.DMA1),
        p.GPIOA,
        p.GPIOB,
        p.GPIOC,
        p.RCC,
    );
    pwm.enable(Channel::_1);

    // end of frame
    *r.BUFFER.borrow_mut().last_mut().unwrap() = 0;

    // set each RGB value to 0x0A0A0A
    for byte in r.BUFFER.borrow_mut()[..(24 * 24)].chunks_mut(8) {
        byte.copy_from_slice(&[_0, _0, _0, _0, _1, _1, _1, _1]);
    }
}

fn idle(_t: &mut Threshold, r: idle::Resources) -> ! {
    let pwm = Pwm(&*r.TIM3);
    let buffer = Static::wrap_mut(r.BUFFER);

    pwm.set_duties(r.DMA1, Channel::_1, buffer).unwrap();

    block!(buffer.release(r.DMA1)).unwrap();

    rtfm::bkpt();

    loop {
        rtfm::wfi();
    }
}
