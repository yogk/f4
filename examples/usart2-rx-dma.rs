//! Test receiving serial data using the DMA. Program will stop at breakpoint after 8 bytes are received.
#![deny(unsafe_code)]
#![deny(warnings)]
#![feature(const_fn)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m_rtfm as rtfm;
extern crate f4;

use f4::Serial;
use f4::dma::{Buffer, Dma1Channel5};
use f4::time::Hertz;
use rtfm::{app, Threshold};

const BAUD_RATE: Hertz = Hertz(115_200);

app! {
    device: f4::stm32f40x,

    resources: {
        static BUFFER: Buffer<[u8; 8], Dma1Channel5> = Buffer::new([0; 8]);
    },

    tasks: {
        DMA1_STREAM5: {
            path: transfer_done,
            resources: [BUFFER, DMA1],
        },
    },
}

fn init(p: init::Peripherals, r: init::Resources) {
    let serial = Serial(p.USART2);

    serial.init(BAUD_RATE.invert(), Some(p.DMA1), p.GPIOA, p.RCC);

    serial.read_exact(p.DMA1, r.BUFFER).unwrap();
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}

fn transfer_done(_t: &mut Threshold, r: DMA1_STREAM5::Resources) {
    r.BUFFER.release(r.DMA1).unwrap();

    rtfm::bkpt();
}
