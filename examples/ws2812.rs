//! Drive a ring of 24 WS2812 LEDs
//!
//! To test this demo connect the data-in pin of the LED ring to pin PA0
#![deny(unsafe_code)]
// #![deny(warnings)]
#![feature(const_fn)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m_rtfm as rtfm;
extern crate f4;
#[macro_use]
extern crate nb;

use f4::dma::{Buffer, Dma1Stream5};
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
        static BUFFER: Buffer<[u8; 577], Dma1Stream5> = Buffer::new([_0; 577]);
        // static X: u8 = 0;
    },

    idle: {
        resources: [BUFFER, DMA1, TIM2],
    },

    // tasks: {
    //     DMA1_STREAM5: {
    //         path: transfer_done,
    //          resources: [X],
    //     },
    // },
}

fn init(p: init::Peripherals, r: init::Resources) {
    let pwm = Pwm(p.TIM2);

    pwm.init(
        FREQUENCY.invert(),
        Channel::_1,
        Some(p.DMA1),
        p.GPIOA,
        p.GPIOB,
        p.GPIOC,
        p.RCC,
    );
    // pwm.set_duty(Channel::_1, pwm.get_max_duty() / 2);
    pwm.enable(Channel::_1);

    // end of frame
    *r.BUFFER.borrow_mut().last_mut().unwrap() = 0;

    // set each RGB value to 0x0A0A0A
    for byte in r.BUFFER.borrow_mut()[..(24 * 24)].chunks_mut(8) {
        byte.copy_from_slice(&[_0, _0, _0, _0, _1, _1, _1, _1]);
    }
    // rtfm::bkpt();
}

fn idle(_t: &mut Threshold, r: idle::Resources) -> ! {
    let pwm = Pwm(&*r.TIM2);
    let buffer = Static::wrap_mut(r.BUFFER);

    pwm.set_duties(r.DMA1, Channel::_1, buffer).unwrap();

    block!(buffer.release(r.DMA1)).unwrap();

    loop {
        rtfm::wfi();
    }
}

// fn transfer_done(_t: &mut Threshold, _r: DMA1_STREAM5::Resources) {
//     rtfm::bkpt();
// }
