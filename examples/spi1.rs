//! Interfacing the LSM9DS1 3D accelerometer, gyroscope, and magnetometer
//! using SPI3.
//!
//! Using the SparkFun LSM9DS1 breakout board, connect:
//! PA_8 -> CS_AG            (Chip Select for accelerometer/gyro)
//! PA_9 -> CS_M             (Chip Select for magnetometer)
//! PB_3 -> SCL              (SPI clock)
//! PB_4 -> SDO_M and SDO_AG (Combined SPI MISO)
//! PB_5 -> SDA              (SPI MOSI)

#![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate f4;
extern crate stm32f40x;

use f4::Spi;
use f4::prelude::*;
use rtfm::{app, Threshold};
use stm32f40x::{GPIOA};

app! {
    device: f4::stm32f40x,

    idle: {
        resources: [SPI3, GPIOA],
    },
}

fn init(p: init::Peripherals) {
    // Setup CS pins
    {
        // Power on GPIOA
        p.RCC.ahb1enr.modify(|_, w| w.gpioaen().set_bit());
        // Set PA_8 and PA_9 as outputs
        p.GPIOA
            .moder
            .modify(|_, w| w.moder8().bits(1).moder9().bits(1));
        // Use pull-ups
        p.GPIOA
            .pupdr
            .modify(|_, w| unsafe { w.pupdr8().bits(1).pupdr9().bits(1) });
        // Highest output speed
        p.GPIOA
            .ospeedr
            .modify(|_, w| w.ospeedr8().bits(3).ospeedr9().bits(3));
        // Default to high (CS disabled)
        p.GPIOA
            .odr
            .modify(|_, w| w.odr8().bit(true).odr9().bit(true));
    }

    // Init the SPI peripheral
    let spi = Spi(p.SPI3);
    spi.init(p.GPIOA, p.GPIOB, p.RCC);

    // For the LSM9DS1, the second clock transition is
    // the first data capture edge
    // RM0368 20.5.1
    p.SPI3.cr1.modify(|_, w| w.cpha().set_bit());
}

fn enable_m(gpioa: &mut GPIOA) {
    gpioa.odr.modify(|_, w| w.odr9().bit(false));
}

fn disable_m(gpioa: &mut GPIOA) {
    gpioa.odr.modify(|_, w| w.odr9().bit(true));
}

fn enable_ag(gpioa: &mut GPIOA) {
    gpioa.odr.modify(|_, w| w.odr8().bit(false));
}

fn disable_ag(gpioa: &mut GPIOA) {
    gpioa.odr.modify(|_, w| w.odr8().bit(true));
}

fn send_receive(spi: &f4::Spi<f4::stm32f40x::SPI3>, addr: u8) -> u8 {
    // Send and receive using the hal crate
    while spi.send(addr).is_err() {}
    while spi.read().is_err() {}
    while spi.send(0).is_err() {}
    loop {
        if let Ok(byte) = spi.read() {
            break byte;
        }
    }
}

fn to_read_address(sub_address: u8) -> u8 {
    0x80 | (sub_address & 0x3F)
}

fn idle(_t: &mut Threshold, r: idle::Resources) -> ! {
    // Registers to read
    const WHO_AM_I_AG: u8 = 0x0F;
    const WHO_AM_I_M: u8 = 0x0F;

    // Expected answers
    const ANS_AG: u8 = 0x68;
    const ANS_M: u8 = 0x3D;

    let spi = Spi(&*r.SPI3);
    spi.enable();

    // Read accelerometer/gyro identity
    enable_ag(r.GPIOA);
    let ans_ag = send_receive(&spi, to_read_address(WHO_AM_I_AG));
    disable_ag(r.GPIOA);
    // Read magnetometer identity
    enable_m(r.GPIOA);
    let ans_m = send_receive(&spi, to_read_address(WHO_AM_I_M));
    disable_m(r.GPIOA);

    spi.disable();

    // Program will panic if these are incorrect
    assert_eq!(ans_ag, ANS_AG);
    assert_eq!(ans_m, ANS_M);

    loop {
        rtfm::wfi();
    }
}
