#![feature(proc_macro)]
#![no_std]

extern crate cortex_m_rtfm as rtfm;
extern crate f3;
#[macro_use]
extern crate nb;

use f3::prelude::*;
use f3::{Serial, stm32f30x};
use rtfm::{app, Threshold};

app! {
    device: stm32f30x,

    resources: {
        static SERIAL: Serial;
    },

    idle: {
        resources: [SERIAL],
    },
}

fn init(p: init::Peripherals) -> init::LateResourceValues {
    p.device.RCC.ahbenr.modify(|_, w| w.iopaen().enabled());
    p.device.RCC.apb2enr.modify(|_, w| w.usart1en().enabled());

    let mut gpioa = p.device.GPIOA.pins();

    let tx = gpioa.PA9.af7(&mut gpioa.AFRH, &mut gpioa.MODER);
    let rx = gpioa.PA10.af7(&mut gpioa.AFRH, &mut gpioa.MODER);

    let serial = Serial::new(p.device.USART1, (tx, rx), 115_200.bps());

    init::LateResourceValues { SERIAL: serial }
}

fn idle(_t: &mut Threshold, r: idle::Resources) -> ! {
    for byte in b"Hello, world!\n".iter() {
        block!(r.SERIAL.write(*byte)).unwrap();
    }

    rtfm::bkpt();

    loop {
        rtfm::wfi();
    }
}
