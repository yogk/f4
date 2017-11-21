#![feature(proc_macro)]
#![no_std]

extern crate cortex_m_rtfm as rtfm;
extern crate f3;

use f3::prelude::*;
use f3::stm32f30x;
use rtfm::app;

app! {
    device: stm32f30x,
}

fn init(p: init::Peripherals) {
    p.device.RCC.ahbenr.modify(|_, w| w.iopeen().enabled());

    let mut gpioe = p.device.GPIOE.pins();

    gpioe.PE9.as_output(&mut gpioe.MODER).high();
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}
