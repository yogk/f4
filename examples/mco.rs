// #![deny(unsafe_code)]
// #![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m_rtfm as rtfm;
#[macro_use]
extern crate f4;

use rtfm::app;
use f4::led::{self, LED};
use f4::clock;

// TASKS & RESOURCES
app! {
    device: f4::stm32f40x,
}

// INITIALIZATION PHASE
fn init(p: init::Peripherals) {
    // RM0368 6.2.10
    // Configure PA8 as MCO_1 alternate function to output HSI clock
    p.RCC.ahb1enr.modify(|_, w| w.gpioaen().set_bit()); //Enable GPIOA clock
    p.GPIOA.ospeedr.modify(|_,w| w.ospeedr8().bits(0b11)); //Highest output speed
    p.GPIOA.afrh.modify(|_,w| w.afrh8().bits(0)); //Alternate function AF0 MCO_1 on pin 8
    p.GPIOA.moder.modify(|_,w| w.moder8().bits(0b10)); //Alternate function push-pull
    p.RCC.cfgr.modify(|_, w| unsafe {w.mco1().bits(0b00)}); //HSI clock selected
    p.RCC.cfgr.modify(|_, w| unsafe {w.mco1pre().bits(0)}); //No division

    // Configure PC9 as MCO_2 alternate function to output System clock
    p.RCC.ahb1enr.modify(|_, w| w.gpiocen().set_bit()); //Enable GPIOC clock
    p.GPIOC.ospeedr.modify(|_,w| unsafe {w.ospeedr9().bits(0b11)}); //Highest output speed
    p.GPIOC.afrh.modify(|_,w| unsafe {w.afrh9().bits(0)}); //Alternate function AF0 MCO_2 on pin 9
    p.GPIOC.moder.modify(|_,w| unsafe {w.moder9().bits(0b10)}); //Alternate function push-pull
    p.RCC.cfgr.modify(|_, w| unsafe {w.mco2().bits(0b00)}); //MCO2 SYSCLK clock selected
    p.RCC.cfgr.modify(|_, w| unsafe {w.mco2pre().bits(0b110)}); //Divide SYSCLK by 4

    let sysclk = clock::set_100_mhz(&p.RCC, &p.FLASH);
    println!("SYSCLK set to {}", sysclk);
    // PC9 should now output sysclk/4 MHz and PA8 the frequency of the HSI RC

    led::init(&p.GPIOA, &p.RCC);
}

// IDLE LOOP
fn idle() -> ! {
    LED.on();
    // Sleep
    loop {
        rtfm::wfi();
    }
}
