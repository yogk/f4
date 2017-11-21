#![feature(proc_macro)]
#![no_std]

extern crate cortex_m_rtfm as rtfm;
extern crate f3;
#[macro_use]
extern crate nb;

use f3::prelude::*;
use f3::{Timer, stm32f30x};
use rtfm::{app, Threshold};
use f3::gpio::{Output, PE9};

app! {
    device: stm32f30x,

    resources: {
        static LD3: Output<PE9>;
        static TIMER: Timer;
    },

    idle: {
        resources: [LD3, TIMER],
    },
}

fn init(p: init::Peripherals) -> init::LateResourceValues {
    p.device.RCC.ahbenr.modify(|_, w| w.iopeen().enabled());
    p.device.RCC.apb1enr.modify(|_, w| w.tim6en().enabled());

    let mut gpioe = p.device.GPIOE.pins();

    let ld3 = gpioe.PE9.as_output(&mut gpioe.MODER);
    let timer = Timer::new(p.device.TIM6, 1.s());

    init::LateResourceValues {
        LD3: ld3,
        TIMER: timer,
    }
}

fn idle(_t: &mut Threshold, r: idle::Resources) -> ! {
    let mut state = false;

    r.TIMER.resume();
    loop {
        block!(r.TIMER.wait()).unwrap();
        state = !state;

        if state {
            r.LD3.high()
        } else {
            r.LD3.low()
        }
    }
}
