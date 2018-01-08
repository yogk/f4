//! Read from ADC1 input pins PA0, PA1, PA4, PB0, PC1, PC0 (Arduino standard)
//! using ADC Scan Mode with DMA in circular mode.
//! Conversions are triggered by PWM from TIM2 channel 2, rising edge.
//! The 16 bit values are output to the ITM port.

#![deny(unsafe_code)]
#![deny(warnings)]
#![feature(const_fn)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m;
#[macro_use]
extern crate cortex_m_debug;
extern crate cortex_m_rtfm as rtfm;
extern crate f4;

use f4::adc::{Adc, AdcChannel};
use f4::dma::{CircBuffer, Dma2Stream0};
use f4::time::Hertz;
use f4::{Channel, Pwm};
use f4::led::{self, LED};
use rtfm::{app, Threshold};

// FIXME: Sampling is twice as fast when connected to ITM by gdb???
const FREQUENCY: Hertz = Hertz(8);
// The number of ADC input channels to sample
const N: usize = 6;

app! {
    device: f4::stm32f40x,

    resources: {
        static BUFFER: CircBuffer<[u16; N], Dma2Stream0> = CircBuffer::new([[0; N]; 2]);
    },

    tasks: {
        DMA2_STREAM0: {
            path: transfer_done,
            resources: [BUFFER, DMA2],
        },
    },
}

fn init(p: init::Peripherals, r: init::Resources) {
    led::init(p.GPIOA, p.RCC);

    let pwm = Pwm(p.TIM2);
    pwm.init(
        FREQUENCY.invert(),
        Channel::_2,
        None,
        p.GPIOA,
        p.GPIOB,
        p.GPIOC,
        p.RCC,
    );

    let adc = Adc(p.ADC1);

    adc.init(p.DMA2, p.RCC);
    adc.enable_input(AdcChannel::_0, 1, p.GPIOA, p.GPIOB, p.GPIOC);
    adc.enable_input(AdcChannel::_1, 2, p.GPIOA, p.GPIOB, p.GPIOC);
    adc.enable_input(AdcChannel::_4, 3, p.GPIOA, p.GPIOB, p.GPIOC);
    adc.enable_input(AdcChannel::_8, 4, p.GPIOA, p.GPIOB, p.GPIOC);
    adc.enable_input(AdcChannel::_11, 5, p.GPIOA, p.GPIOB, p.GPIOC);
    adc.enable_input(AdcChannel::_10, 6, p.GPIOA, p.GPIOB, p.GPIOC);
    adc.enable();
    adc.start(r.BUFFER, p.DMA2, pwm).unwrap();
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}

fn transfer_done(_t: &mut Threshold, r: DMA2_STREAM0::Resources) {
    match r.BUFFER.read(r.DMA2, |x| {
        let buf: [u16; N] = x.clone();
        buf
    }) {
        Err(_) => rtfm::bkpt(),
        Ok(b) => {
            ipln!("{}, {}, {}, {}, {}, {}", b[0], b[1], b[2], b[3], b[4], b[5]);
            LED.toggle();
        }
    }
}
