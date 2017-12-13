// #![deny(unsafe_code)]
#![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m_rtfm as rtfm;
#[macro_use]
extern crate f4;

use rtfm::app;
use f4::led::{self, LED};
use f4::frequency::apb1;

// TASKS & RESOURCES
app! {
    device: f4::stm32f40x,
}

fn clk_init(p: &init::Peripherals) {
    // setting up the flash memory latency
    // RM0368 8.4.1 (register), 3.4 Table 6
    // we assume 3.3 volt operation, thus 2 cycles for 84mHz
    // apb1 will be at 42 MHz
    ::apb1::set_frequency(42_000_000);
    p.FLASH.acr.modify(|_, w| unsafe { w.latency().bits(2) });
    println!("Flash latency! {:x}", p.FLASH.acr.read().latency().bits());

    p.RCC
        .cfgr
        .modify(|_, w| w.sw0().clear_bit().sw1().clear_bit()); //Switch to HSI
    p.RCC.cfgr.modify(|_, w| unsafe { w.ppre1().bits(4) }); //Configure apb1 prescaler = 2,
    p.RCC.apb1enr.modify(|_, w| w.pwren().set_bit());
    p.RCC.cr.write(|w| w.pllon().clear_bit());

    //Enable PLL
    //					   PP	  PLLN	    PLLM
    // 0b0000 0000 0000 00 01 0 101010000 010000
    // RM0368 6.3.2
    // PP 01
    p.RCC
        .pllcfgr
        .write(|w| unsafe { w.bits(0b00000000000000010101010000010000) }); //Configure PLL

    p.RCC.cr.modify(|_, w| w.pllon().set_bit()); //Enable PLL

    while p.RCC.cr.read().pllrdy().bit_is_clear() {}

    p.RCC
        .cfgr
        .modify(|_, w| w.sw0().clear_bit().sw1().set_bit()); //Switch to PLL

    // System configuration controller clock enable
    p.RCC.apb2enr.modify(|_, w| w.syscfgen().set_bit());

    p.RCC.ahb1enr.modify(|_, w| w.gpioaen().set_bit()); //Enable GPIOA clock
    p.RCC.ahb1enr.modify(|_, w| w.gpioben().set_bit()); //Enable GPIOB clock
}

// INITIALIZATION PHASE
fn init(p: init::Peripherals) {

    // Configure PC9 as MCO_2 alternate function to output SYSCLK
    p.RCC.ahb1enr.modify(|_, w| w.gpiocen().set_bit()); //Enable GPIOC clock
    p.GPIOC.ospeedr.modify(|_,w| unsafe {w.ospeedr9().bits(0b11)}); //Highest output speed
    p.GPIOC.afrh.modify(|_,w| unsafe {w.afrh9().bits(0)}); //Alternate function AF00 MCO_2 on pin 9
    p.GPIOC.moder.modify(|_,w| unsafe {w.moder9().bits(0b10)}); //Alternate function push-pull
    p.RCC.cfgr.modify(|_, w| unsafe {w.mco2().bits(0)}); //MCO2 SYSCLK clock selected
    p.RCC.cfgr.modify(|_, w| unsafe {w.mco2pre().bits(0b111)}); //MCO2 SYSCLK clock selected

    clk_init(&p);

    led::init(&p.GPIOA, &p.RCC);
}

// IDLE LOOP
fn idle() -> ! {
    // Sleep
    loop {
        LED.on();
        rtfm::wfi();
        LED.off();
    }
}
