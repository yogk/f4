#![feature(proc_macro)]
#![no_std]

#[macro_use]
extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate f3;

use f3::stm32f30x;
use rtfm::app;

app! {
    device: stm32f30x,
}

fn init(p: init::Peripherals) {
    iprintln!(&p.core.ITM.stim[0], "Hello, world!");
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}
