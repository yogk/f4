//! Reading and writing to an external EEPROM (Microchip 24LC64) using I2C

// #![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

#[macro_use]
extern crate f4;

extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate stm32f40x;

use f4::I2c;
use f4::prelude::*;
use rtfm::{app, Threshold};

const WRITE: u8 = 0;
const READ: u8 = 1;

app! {
    device: f4::stm32f40x,

    idle: {
        resources: [I2C1],
    },
}

fn init(p: init::Peripherals) {
    // Init the I2C peripheral
    let i2c = I2c(p.I2C1);
    i2c.init(p.GPIOA, p.GPIOB, p.RCC);
    i2c.enable();
    rtfm::bkpt();
}

fn idle(_t: &mut Threshold, r: idle::Resources) -> ! {
    let i2c = I2c(r.I2C1);
    loop {
        let r = i2c.start(0xa0);
        println!("Hello, world! {:?}", r);

        // i2c.write(0x0a).unwrap();
        // i2c.write(0xaa).unwrap();
        // i2c.stop().unwrap();
        rtfm::bkpt();
        // rtfm::wfi();
    }
}
