//! Reading and writing to an external EEPROM (Microchip 24LC64) using I2C

// #![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

#[macro_use(iprint, iprintln)]
extern crate cortex_m;
extern crate cortex_m_debug;
extern crate cortex_m_rtfm as rtfm;
#[macro_use]
extern crate f4;
extern crate stm32f40x;

use f4::I2c;
use rtfm::{app, Threshold};
use core::result::Result;
use stm32f40x::I2C1;
use f4::clock;

const EEPROM_PAGE_SIZE: usize = 32;
const RX_BUFFER_SIZE: usize = 16;

///
#[derive(Debug)]
pub enum Error {
    /// Invalid eeprom memory address
    InvalidMemory,
}

app! {
    device: f4::stm32f40x,

    idle: {
        resources: [I2C1, ITM],
    },
}

fn init(p: init::Peripherals) {
    clock::set_84_mhz(&p.RCC, &p.FLASH);

    // Init the I2C peripheral
    let i2c = I2c(p.I2C1);
    i2c.init(p.GPIOA, p.GPIOB, p.RCC);
    i2c.enable();
    rtfm::bkpt();
}

fn read_eeprom(
    i2c: &I2c<I2C1>,
    mem_addr: u16,
    rx_buffer: &mut [u8; RX_BUFFER_SIZE],
) -> Result<(), Error> {
    if mem_addr > 0x1ff - 32 {
        return Err(Error::InvalidMemory);
    }
    // Write device address and memory address to set eeprom internal cursor
    while i2c.start(0xa0).is_err() {}
    while i2c.write((mem_addr >> 8) as u8).is_err() {}
    while i2c.write(mem_addr as u8).is_err() {}

    // Read incoming bytes and ACK them
    while i2c.start(0xa1).is_err() {}
    for i in 0..RX_BUFFER_SIZE - 1 {
        rx_buffer[i] = loop {
            if let Ok(byte) = i2c.read_ack() {
                break byte;
            }
        }
    }
    // Do not ACK the last byte received and send STOP
    rx_buffer[RX_BUFFER_SIZE - 1] = loop {
        if let Ok(byte) = i2c.read_nack() {
            break byte;
        }
    };
    Ok(())
}

fn write_eeprom(
    i2c: &I2c<I2C1>,
    mem_addr: u16,
    tx_buffer: &[u8; EEPROM_PAGE_SIZE],
) -> Result<(), Error> {
    if mem_addr > 0x1ff - EEPROM_PAGE_SIZE as u16 || mem_addr % EEPROM_PAGE_SIZE as u16 != 0 {
        return Err(Error::InvalidMemory);
    }
    // Write device address and memory address to set eeprom internal cursor
    while i2c.start(0xa0).is_err() {}
    while i2c.write((mem_addr >> 8) as u8).is_err() {}
    while i2c.write(mem_addr as u8).is_err() {}

    // Write data
    for i in 0..EEPROM_PAGE_SIZE {
        while i2c.write(tx_buffer[i]).is_err() {}
    }
    while i2c.stop().is_err() {}
    Ok(())
}

fn idle(_t: &mut Threshold, r: idle::Resources) -> ! {
    let i2c = I2c(r.I2C1);
    let mut rx: [u8; RX_BUFFER_SIZE] = [0; RX_BUFFER_SIZE];
    let mut tx: [u8; EEPROM_PAGE_SIZE] = [
        0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, //
        0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x10, //
        0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, //
        0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 0x20, //
    ];
    // Write in 32 byte pages
    for page in 0..4 {
        let mem_addr: u16 = page * EEPROM_PAGE_SIZE as u16;
        write_eeprom(&i2c, mem_addr, &tx).unwrap();
    }

    // // Read back and print
    // let mut mem_addr = 0x0000;
    // // loop {
    // for page in 0..4 {
    //     match read_eeprom(&i2c, mem_addr, &mut rx) {
    //         Err(_) => mem_addr = 0,
    //         Ok(_) => {
    //             // iprint!(&r.ITM.stim[0], "0x{:04X}: ", mem_addr);
    //             for i in 0..rx.len() {
    //                 iprint!(&r.ITM.stim[0], "0x{:02X} ", rx[i]);
    //             }
    //             iprint!(&r.ITM.stim[0], "\r\n");
    //             // iprintln!(&r.ITM.stim[0], "");
    //             // rtfm::bkpt();
    //             mem_addr += RX_BUFFER_SIZE as u16;
    //         }
    //     }
    // }
    loop {}
}
