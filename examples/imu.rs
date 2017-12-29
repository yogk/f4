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
// #![deny(warnings)]
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
use f4::dma::{Buffer, Dma1Channel5, Dma1Channel6};
use f4::time::Hertz;
use f4::clock;
use f4::timer::Timer;
use f4::prelude::*;
use f4::math_utils::{Quaternion, Vector3};
use f4::madgwick_ahrs::MadgwickAhrs;
use rtfm::{app, Threshold};

const BAUD_RATE: Hertz = Hertz(115_200);
const SAMPLE_FREQUENCY: u32 = 100;
const SERIAL_FREQUENCY: u32 = 10;
const MAX_TX_LEN: usize = 256;

app! {
    device: f4::stm32f40x,

    resources: {
        static ACC: Vector3<f32> = Vector3::<f32>::new();
        static GYRO: Vector3<f32> = Vector3::<f32>::new();
        static MAG: Vector3<f32> = Vector3::<f32>::new();
        static ORIENTATION: Quaternion<f32> = Quaternion::<f32>::new();
        static IMU_SETTINGS: ImuSettings = ImuSettings::new();
        static FILTER: MadgwickAhrs = MadgwickAhrs::begin(SAMPLE_FREQUENCY as f32);
        static TX_BUFFER: Buffer<[u8; MAX_TX_LEN], Dma1Channel6> = Buffer::new([0; MAX_TX_LEN]);
        static RX_BUFFER: Buffer<[u8; 1], Dma1Channel5> = Buffer::new([0; 1]);
        static CMD: u8 = b'q';
    },

    tasks: {
        SYS_TICK: {
            path: sys_tick,
            priority: 1,
            resources: [ACC, GYRO, MAG, ORIENTATION, TX_BUFFER, CMD, DMA1, USART2],
        },
        DMA1_STREAM5: {
            path: rx_done,
            priority: 2,
            resources: [RX_BUFFER, CMD, DMA1, USART2],
        },
        DMA1_STREAM6: {
            path: tx_done,
            priority: 4,
            resources: [TX_BUFFER, DMA1],
        },
        TIM2: {
            path: sample_imu,
            priority: 3,
            resources: [ACC, GYRO, MAG, ORIENTATION, IMU_SETTINGS, FILTER, SPI3, GPIOA, TIM2],
        },
    },
}

fn sample_imu(_t: &mut Threshold, r: TIM2::Resources) {
    let spi = Spi(&**r.SPI3);
    let imu = Lsm9ds1(&**r.SPI3);
    r.ACC.set(imu.read_acc(&spi, r.GPIOA, &r.IMU_SETTINGS));
    r.GYRO.set(imu.read_gyro(&spi, r.GPIOA, &r.IMU_SETTINGS));
    r.MAG.set(imu.read_mag(&spi, r.GPIOA, &r.IMU_SETTINGS));
    r.ORIENTATION.set(r.FILTER.madgwick_ahrs_update(
        r.GYRO.x/2.0,
        r.GYRO.y/2.0,
        r.GYRO.z/2.0,
        -r.ACC.x,
        -r.ACC.y,
        r.ACC.z,
        r.MAG.x,
        r.MAG.y,
        r.MAG.z,
    ));

    r.TIM2.sr.modify(|_, w| w.uif().clear_bit());
}
fn rx_done(t: &mut Threshold, r: DMA1_STREAM5::Resources) {
    use rtfm::Resource;
    **r.CMD = r.RX_BUFFER.claim(t, |rx, t| {
        // We need to unlock the DMA to use it again in the future
        r.DMA1.claim(t, |dma, _| rx.release(dma).unwrap());
        // Read the single character in the input buffer
        let byte = rx.deref().borrow()[0];
        r.USART2.claim(t, |usart, t| {
            // Get ready to receive again
            let serial = Serial(&**usart);
            r.DMA1
                .claim(t, |dma, _| serial.read_exact(dma, rx).unwrap());
        });
        byte
    })
}

fn sys_tick(t: &mut Threshold, mut r: SYS_TICK::Resources) {
    use rtfm::Resource;
    let a = r.ACC.claim(t, |v, _| **v.clone());
    let g = r.GYRO.claim(t, |v, _| **v.clone());
    let m = r.MAG.claim(t, |v, _| **v.clone());
    let q = r.ORIENTATION.claim(t, |v, _| **v.clone());
    let e = q.to_euler_angles();

    let cmd = r.CMD.claim(t, |cmd, _| **cmd);
    match cmd {
        b'a' => {
            uprint!(
                t,
                r.USART2,
                r.DMA1,
                r.TX_BUFFER,
                "{} {} {}\r\n",
                a.x,
                a.y,
                a.z,
            );
        }
        b'g' => {
            uprint!(
                t,
                r.USART2,
                r.DMA1,
                r.TX_BUFFER,
                "{} {} {}\r\n",
                g.x,
                g.y,
                g.z,
            );
        }
        b'm' => {
            uprint!(
                t,
                r.USART2,
                r.DMA1,
                r.TX_BUFFER,
                "{} {} {}\r\n",
                m.x,
                m.y,
                m.z
            );
        }
        b'e' => {
            uprint!(
                t,
                r.USART2,
                r.DMA1,
                r.TX_BUFFER,
                "Orientation: {} {} {}\r\n",
                e.x,
                e.y,
                e.z
            );
        }
        b'q' => {
            uprint!(
                t,
                r.USART2,
                r.DMA1,
                r.TX_BUFFER,
                "Quaternion: {} {} {} {}\r\n",
                q.x,
                q.y,
                q.z,
                q.w,
            );
        }
        _ => (),
    }
}

fn init(p: init::Peripherals, r: init::Resources) {
    let clk = clock::set_84_mhz(&p.RCC, &p.FLASH);
    p.SYST.set_clock_source(SystClkSource::Core);
    p.SYST.set_reload(clk / SERIAL_FREQUENCY);
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

    // Reset the IMU to a known state and initialize the sensors.
    let imu = Lsm9ds1(p.SPI3);
    imu.reset(&spi, &p.GPIOA);
    imu.init_gyro(&spi, &p.GPIOA, &r.IMU_SETTINGS);
    imu.init_acc(&spi, &p.GPIOA, &r.IMU_SETTINGS);
    imu.init_mag(&spi, &p.GPIOA, &r.IMU_SETTINGS);

    let timer = Timer(&*p.TIM2);
    timer.init(Hertz(SAMPLE_FREQUENCY).invert(), p.RCC);
    timer.resume();

    // Listen to serial input on the receive DMA
    serial.read_exact(p.DMA1, r.RX_BUFFER).unwrap();
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
