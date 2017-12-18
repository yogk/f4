//! Set the clock frequency on the Nucleo-64 stm32f40x
#![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate f4;

use rtfm::{app, Threshold};
use f4::led::{self, LED};
use f4::clock;
use cortex_m::peripheral::SystClkSource;

const FREQUENCY: u32 = 10; // Hz

// TASKS & RESOURCES
app! {
    device: f4::stm32f40x,

    resources: {
        static ON: bool = false;
    },

    tasks: {
        SYS_TICK: {
            path: toggle,
            resources: [ON],
        },
    },
}

// INITIALIZATION PHASE
fn init(p: init::Peripherals, _r: init::Resources) {
    // See RM0368 6.2.10 Clock-out capability
    // PC9 outputs SYSCLK/4 MHz and PA8 the frequency of the HSI RC

    // Configure PA8 as MCO_1 alternate function to output HSI clock
    p.RCC.ahb1enr.modify(|_, w| w.gpioaen().set_bit()); //Enable GPIOA clock
    p.GPIOA.ospeedr.modify(|_, w| w.ospeedr8().bits(0b11)); //Highest output speed
    p.GPIOA.afrh.modify(|_, w| w.afrh8().bits(0)); //Alternate function AF0 MCO_1 on pin 8
    p.GPIOA.moder.modify(|_, w| w.moder8().bits(0b10)); //Alternate function push-pull
    p.RCC.cfgr.modify(|_, w| unsafe { w.mco1().bits(0b00) }); //HSI clock selected
    p.RCC.cfgr.modify(|_, w| unsafe { w.mco1pre().bits(0) }); //No division

    // Configure PC9 as MCO_2 alternate function to output System clock
    p.RCC.ahb1enr.modify(|_, w| w.gpiocen().set_bit()); //Enable GPIOC clock
    p.GPIOC
        .ospeedr
        .modify(|_, w| unsafe { w.ospeedr9().bits(0b11) }); //Highest output speed
    p.GPIOC.afrh.modify(|_, w| unsafe { w.afrh9().bits(0) }); //Alternate function AF0 MCO_2 on pin 9
    p.GPIOC
        .moder
        .modify(|_, w| unsafe { w.moder9().bits(0b10) }); //Alternate function push-pull
    p.RCC.cfgr.modify(|_, w| unsafe { w.mco2().bits(0b00) }); //MCO2 SYSCLK clock selected
    p.RCC.cfgr.modify(|_, w| unsafe { w.mco2pre().bits(0b110) }); //Divide SYSCLK by 4

    // Set the clock to 84 MHz for compatibility with stm32f401
    // let clk = clock::set_84_mhz(&p.RCC, &p.FLASH);

    // The stm32f411 supports 100 MHz.
    let clk = clock::set_100_mhz(&p.RCC, &p.FLASH);

    // We can also use a lower frequency by providing valid PLL constants.
    // Since the HSI RC is 16 MHz, we get 16/8*50/4 = 25 MHz
    // let clk = clock::set(&p.RCC, &p.FLASH, 8, 50, 4);

    // Light the green LED when we start idling.
    led::init(&p.GPIOA, &p.RCC);

    p.SYST.set_clock_source(SystClkSource::Core);
    p.SYST.set_reload(clk / FREQUENCY);
    p.SYST.enable_interrupt();
    p.SYST.enable_counter();
}

// TASKS
// Toggle the state of the LED
fn toggle(_t: &mut Threshold, r: SYS_TICK::Resources) {
    **r.ON = !**r.ON;

    if **r.ON {
        LED.on();
    } else {
        LED.off();
    }
}

// IDLE LOOP
fn idle() -> ! {
    // Sleep
    loop {
        rtfm::wfi();
    }
}
