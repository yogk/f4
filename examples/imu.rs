//! Interfacing the LSM9DS1 3D accelerometer, gyroscope, and magnetometer
//! using SPI3.
//!
//! Using the SparkFun LSM9DS1 breakout board, connect:
//! PA_8 -> CS_AG            (Chip Select for accelerometer/gyro)
//! PA_9 -> CS_M             (Chip Select for magnetometer)
//! PB_3 -> SCL              (SPI clock)
//! PB_4 -> SDO_M and SDO_AG (Combined SPI MISO)
//! PB_5 -> SDA              (SPI MOSI)
#![deny(unsafe_code)]
#![deny(warnings)]
#![feature(const_fn)]
#![feature(proc_macro)]
#![feature(lang_items)]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
#[macro_use]
extern crate f4;
extern crate heapless;
extern crate stm32f40x;

use cortex_m::peripheral::SystClkSource;
use core::ops::Deref;
use f4::lsm9ds1::{ImuSettings, Lsm9ds1};
use f4::Serial;
use f4::Spi;
use f4::dma::{Buffer, Dma1Channel6};
use f4::time::Hertz;
use f4::clock;
use rtfm::{app, Threshold};

const BAUD_RATE: Hertz = Hertz(115_200);
const FREQUENCY: u32 = 10;
const MAX_TX_LEN: usize = 256;

app! {
    device: f4::stm32f40x,

    resources: {
        static IMU_SETTINGS: ImuSettings = ImuSettings::new();
        static TX_BUFFER: Buffer<[u8; MAX_TX_LEN], Dma1Channel6> = Buffer::new([0; MAX_TX_LEN]);
    },

    tasks: {
        SYS_TICK: {
            path: sys_tick,
            priority: 1,
            resources: [TX_BUFFER, IMU_SETTINGS, DMA1, USART2, SPI3, GPIOA],
        },
        DMA1_STREAM6: {
            path: tx_done,
            priority: 2,
            resources: [TX_BUFFER, DMA1],
        },
    },
}

fn sys_tick(t: &mut Threshold, mut r: SYS_TICK::Resources) {
    let spi = Spi(&**r.SPI3);
    let imu = Lsm9ds1(&**r.SPI3);
    let g = imu.read_gyro(&spi, r.GPIOA, &r.IMU_SETTINGS);
    let a = imu.read_acc(&spi, r.GPIOA, &r.IMU_SETTINGS);
    let m = imu.read_mag(&spi, r.GPIOA, &r.IMU_SETTINGS);

    uprint!(
        t,
        r.USART2,
        r.DMA1,
        r.TX_BUFFER,
        "a=({0: <11}, {1: <11}, {2: <11}), g=({3: <11}, {4: <11}, {5: <11}), m=({6: <11}, {7: <11}, {8: <11})\r\n",
        a.x,a.y,a.z,g.x,g.y,g.z,m.x,m.y,m.z
    );
}

fn init(p: init::Peripherals, r: init::Resources) {
    let clk = clock::set_84_mhz(&p.RCC, &p.FLASH);
    p.SYST.set_clock_source(SystClkSource::Core);
    p.SYST.set_reload(clk / FREQUENCY);
    p.SYST.enable_interrupt();
    p.SYST.enable_counter();

    // Start the serial port
    let serial = Serial(p.USART2);
    serial.init(BAUD_RATE.invert(), Some(p.DMA1), p.GPIOA, p.RCC);

    // Setup CS pins
    {
        // Power on GPIOA
        p.RCC.ahb1enr.modify(|_, w| w.gpioaen().set_bit());
        // Set PA_8 and PA_9 as outputs
        p.GPIOA
            .moder
            .modify(|_, w| w.moder8().bits(1).moder9().bits(1));
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
    spi.enable();

    let imu = Lsm9ds1(p.SPI3);
    imu.init_gyro(&spi, &p.GPIOA, &r.IMU_SETTINGS);
    imu.init_acc(&spi, &p.GPIOA, &r.IMU_SETTINGS);
    imu.init_mag(&spi, &p.GPIOA, &r.IMU_SETTINGS);
}

// Interrupt for serial transmit DMA
fn tx_done(t: &mut Threshold, r: DMA1_STREAM6::Resources) {
    use rtfm::Resource;
    // We need to unlock the DMA to use it again in the future
    r.TX_BUFFER.claim(t, |tx, t| {
        r.DMA1.claim(t, |dma, _| tx.release(dma).unwrap());
    });
    // Clear the transmit buffer so we do not retransmit old stuff
    r.TX_BUFFER.claim_mut(t, |tx, _| {
        let array = &**tx.deref();
        array.borrow_mut()[..MAX_TX_LEN].clone_from_slice(&[0; MAX_TX_LEN]);
    });
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}
