#![feature(proc_macro)]
#![no_std]

extern crate cortex_m_rtfm as rtfm;
extern crate f3;

use f3::prelude::*;
use f3::{Serial, stm32f30x};
use rtfm::app;

app! {
    device: stm32f30x,

    resources: {
        static BUFFER: [u8; 8] = [0; 8];
    },

    init: {
        resources: [BUFFER],
    },
}

fn init(p: init::Peripherals, r: init::Resources) {
    p.device
        .RCC
        .ahbenr
        .modify(|_, w| w.iopaen().enabled().dmaen().enabled());
    p.device.RCC.apb2enr.modify(|_, w| w.usart1en().enabled());

    let mut gpioa = p.device.GPIOA.pins();

    let tx = gpioa.PA9.af7(&mut gpioa.AFRH, &mut gpioa.MODER);
    let rx = gpioa.PA10.af7(&mut gpioa.AFRH, &mut gpioa.MODER);

    let (tx, rx) = Serial::new(p.device.USART1, (tx, rx), 115_200.bps()).split();
    let channels = p.device.DMA1.split();

    // challenge: turn this into a loop
    let (_, (_, buffer)) = rx.read_exact(channels.5, r.BUFFER).wait().unwrap();
    tx.write_all(channels.4, buffer).wait().unwrap();
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}
