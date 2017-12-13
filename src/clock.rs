//! System clocking

use stm32f40x::{FLASH, RCC};

const HSI_FREQ: u32 = 16_000_000;

fn calculate_pll(m: u8, n: u16, p: u8) -> (u32, u32) {
    // RM0368 6.3.2

    let pllm_output = match m {
        2...63 => match HSI_FREQ / m as u32 {
            950_000...2_100_000 => HSI_FREQ / m as u32,
            _ => panic!("Invalid PLLM output frequency"),
        },
        _ => panic!("Invalid PLLM divisor"),
    };
    let vco_clock = match n {
        50...432 => match pllm_output * n as u32 {
            100_000_000...432_000_000 => pllm_output * n as u32,
            _ => panic!("Invalid PLLN output frequency"),
        },
        _ => panic!("Invalid PLLN multiplier"),
    };
    let log2p = match p {
        2 => 0b00,
        4 => 0b01,
        6 => 0b10,
        8 => 0b11,
        _ => panic!("Invalid PLLP divisor"),
    };
    let pll_output = match vco_clock / p as u32 {
        24_000_000...100_000_000 => vco_clock / p as u32,
        _ => panic!("Invalid PLLP output frequency"),
    };
    let pll_bitmask = ((log2p as u32) << 16) | ((n as u32) << 6) | (m as u32);

    (pll_bitmask, pll_output)
}



/// Set system clock
pub fn set(rcc: &RCC, flash: &FLASH, m: u8, n: u16, p: u8) -> u32 {
    let (pll_bitmask, sysclk) = calculate_pll(m, n, p);
    // let ahb prescaler = 1, then
    let hclk = sysclk;

    // setting up the flash memory latency
    // RM0368 8.4.1 (register), 3.4 Table 6
    // apb1 will be at 42 MHz
    rcc.cfgr.modify(|_, w| unsafe { w.ppre1().bits(4) }); //Configure apb1 prescaler = 2,
    ::apb1::set_frequency(hclk / 2);

    // we assume 3.3 volt operation, thus 2 cycles for 84MHz
    flash.acr.modify(|_, w| unsafe {
        w.latency().bits(match hclk {
            0...30_000_000 => 0,
            30_000_000...64_000_000 => 1,
            64_000_000...90_000_000 => 2,
            90_000_000...100_000_000 => 3,
            _ => panic!("Invalid HCLK frequency"),
        })
    });
    // println!("Flash latency! {:x}", p.FLASH.acr.read().latency().bits());

    rcc.cfgr
        .modify(|_, w| w.sw0().clear_bit().sw1().clear_bit()); //Switch to HSI
    rcc.apb1enr.modify(|_, w| w.pwren().set_bit());
    rcc.cr.write(|w| w.pllon().clear_bit());

    //Enable PLL
    rcc.pllcfgr.write(|w| unsafe { w.bits(pll_bitmask) }); //Configure PLL

    rcc.cr.modify(|_, w| w.pllon().set_bit()); //Enable PLL

    while rcc.cr.read().pllrdy().bit_is_clear() {}

    rcc.cfgr.modify(|_, w| w.sw0().clear_bit().sw1().set_bit()); //Switch to PLL

    // System configuration controller clock enable
    rcc.apb2enr.modify(|_, w| w.syscfgen().set_bit());

    rcc.ahb1enr.modify(|_, w| w.gpioaen().set_bit()); //Enable GPIOA clock
    rcc.ahb1enr.modify(|_, w| w.gpioben().set_bit()); //Enable GPIOB clock
    hclk
}

/// Set system clock to 100 MHz
pub fn set_100_mhz(rcc: &RCC, flash: &FLASH) -> u32 {
    set(rcc, flash, 16, 400, 4)
}
/// Set system clock to 84 MHz
pub fn set_84_mhz(rcc: &RCC, flash: &FLASH) -> u32 {
    set(rcc, flash, 16, 336, 4)
}
