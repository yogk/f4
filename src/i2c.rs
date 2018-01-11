//! I2C
//!
//! You can use the I2c interface with these instances
//!
//! # I2C1
//! - SCL = PB8
//! - SDA = PB9
//!
//! # I2C2
//! - SCL = PB10
//! - SDA = PB3
//!
//! # I2C3
//! - SCL = PA8
//! - SDA = PB4

use core::any::{Any, TypeId};
use core::ptr;

use nb;
use stm32f40x::{I2C1, I2C2, I2C3, GPIOA, GPIOB, RCC};

/// I2C result
pub type Result<T> = ::core::result::Result<T, nb::Error<Error>>;

/// I2C error
#[derive(Debug)]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Timeout occurred, SCL remained LOW for 25 ms
    Timeout,
    /// Bus error
    BusError,
    #[doc(hidden)] _Extensible,
}

/// Serial Peripheral Interface
pub struct I2c<'a, T>(pub &'a T)
where
    T: 'a;

/// I2c interface
macro_rules! impl_I2c {
    ($S:ident) => {
        impl<'a> I2c<'a, $S>
        {

            /// Initializes the SPI
            pub fn init(&self,
                    _gpioa: &GPIOA,
                    gpiob: &GPIOB,
                    rcc: &RCC) {
                let i2c = self.0;
                if i2c.get_type_id() == TypeId::of::<I2C1>() {
                    // enable I2C1, GPIOB
                    rcc.apb1enr.modify(|_, w| {
                        w.i2c1en().set_bit()
                    });
                    rcc.ahb1enr.modify(|_, w| {
                        w.gpioben().set_bit()
                    });
                    // DM00102166 - Alternate function AF4, Table 9
                    gpiob.afrh.modify(|_, w| unsafe {
                        w.afrh8().bits(4)
                        .afrh9().bits(4)});
                    // RM0368 8.3 Table 23
                    // Highest output speed
                    gpiob.ospeedr.modify(|_, w| unsafe {
                        w.ospeedr8().bits(0b11)
                        .ospeedr9().bits(0b11)});
                    // Alternate function mode
                    gpiob.moder.modify(|_, w| unsafe {
                        w.moder8().bits(2)
                        .moder9().bits(2)});
                    // Push pull
                    gpiob.otyper.modify(|_, w|
                        w.ot8().set_bit()
                        .ot9().set_bit());
                    // Pull ups
                    gpiob.pupdr.modify(|_, w| unsafe {
                        w.pupdr8().bits(0)
                        .pupdr9().bits(0)});

                } else  if i2c.get_type_id() == TypeId::of::<I2C2>() {
                } else if i2c.get_type_id() == TypeId::of::<I2C3>() {
                }

                self.disable();

                // Peripheral bus frequency in MHz
                let pclk1_mhz: u32 = ::frequency::apb1::Ticks::from(::time::Microseconds(1)).into();
                let pclk1_hz: u32 = ::frequency::apb1::Ticks::from(::time::Seconds(1)).into();

                i2c.cr1.write(|w|  w.swrst().set_bit());
                i2c.cr1.write(|w| unsafe{ w.bits(0) });

                i2c.cr2.modify(|_,w| unsafe { w.freq().bits(pclk1_mhz as u8) });

                let mut result: u16 = (pclk1_hz / (100_000>>1)) as u16; // <<1 for 400_000
                if result == 0 {
                    result = 1;
                }

                i2c.ccr.modify(|_,w| unsafe {
                    w.f_s().clear_bit() // Standard mode I2C
                    .duty().clear_bit() // Fast mode t_low/t_high = 2
                    .ccr().bits(result)
                });
                i2c.trise.modify(|_,w| unsafe { w.trise().bits((pclk1_mhz+1) as u8)});
            }

            /// Disables the I2C bus
            pub fn disable(&self) {
                self.0.cr1.modify(|_, w| w.pe().clear_bit())
            }

            /// Enables the I2C bus
            pub fn enable(&self) {
                self.0.cr1.modify(|_, w| w.pe().set_bit())
            }

            ///
            pub fn read(&self) -> Result<u8> {
                let i2c = self.0;
                let sr = i2c.sr1.read();

                if sr.ovr().bit_is_set() {
                    Err(nb::Error::Other(Error::Overrun))
                } else if sr.timeout().bit_is_set() {
                    Err(nb::Error::Other(Error::Timeout))
                } else if sr.berr().bit_is_set() {
                    Err(nb::Error::Other(Error::BusError))
                } else if sr.rx_ne().bit_is_set() {
                    Ok(unsafe {
                        ptr::read_volatile(&i2c.dr as *const _ as *const u8)
                    })
                } else {
                    Err(nb::Error::WouldBlock)
                }
            }

            ///
            pub fn start(&self, _control_byte: u8)  -> u8 {
                let i2c = self.0;

                let _sr1 = i2c.sr1.read();
                let _sr2 = i2c.sr2.read();

                let bytes: [u8;4] = [0xa0, 0x00, 0x02, 0xa1];

                i2c.cr1.modify(|_,w|  w.ack().set_bit());

                i2c.cr1.modify(|_, w| w.start().set_bit());
                while i2c.sr1.read().sb().bit_is_clear() {}

                unsafe {
                    ptr::write_volatile(&i2c.dr as *const _ as *mut u8, bytes[0]);
                }
                while i2c.sr1.read().addr().bit_is_clear() {}
                while i2c.sr1.read().addr().bit_is_set() {
                    let _sr2 = i2c.sr2.read();
                }
                while i2c.sr1.read().tx_e().bit_is_clear() {}

                unsafe {
                    ptr::write_volatile(&i2c.dr as *const _ as *mut u8, bytes[1]);
                }

                while i2c.sr1.read().tx_e().bit_is_clear() {}

                unsafe {
                    ptr::write_volatile(&i2c.dr as *const _ as *mut u8, bytes[2]);
                }

                while i2c.sr1.read().tx_e().bit_is_clear() {}

                // Read
                i2c.cr1.modify(|_, w| w.start().set_bit());
                while i2c.sr1.read().sb().bit_is_clear() {}

                unsafe {
                    ptr::write_volatile(&i2c.dr as *const _ as *mut u8, bytes[3]);
                }

                i2c.cr1.modify(|_,w|  w.ack().clear_bit()); // if one byte rx
                while i2c.sr1.read().addr().bit_is_clear() {}
                while i2c.sr1.read().addr().bit_is_set() {
                    let _sr2 = i2c.sr2.read();
                }

                while i2c.sr1.read().rx_ne().bit_is_clear() {}
                let r: Result<u8> = Ok(unsafe {
                    ptr::read_volatile(&i2c.dr as *const _ as *const u8)
                });

                i2c.cr1.modify(|_,w|  w.ack().clear_bit());

                i2c.cr1.modify(|_, w| w.stop().set_bit());

                r.unwrap()
            }

           ///
            pub fn stop(&self)  -> Result<()> {
                let i2c = self.0;
                i2c.cr1.modify(|_, w| w.stop().set_bit());
                while i2c.cr1.read().stop().bit_is_set() {}
                Ok(())
            }


            ///
            pub fn write(&self, byte: u8) -> Result<()> {
                let i2c = self.0;

                unsafe {
                    ptr::write_volatile(&i2c.dr as *const _ as *mut u8, byte)
                }

                // if i2c.sr1.read().rx_ne().bit_is_set() {
                //     unsafe {
                //         ptr::read_volatile(&i2c.dr as *const _ as *const u8);
                //     }
                // }
                // while i2c.sr1.read().tx_e().bit_is_clear() {}
                // while i2c.sr1.read().btf().bit_is_clear() {}
                Ok(())

                // if sr.ovr().bit_is_set() {
                //     panic!("p");
                //     // Err(nb::Error::Other(Error::Overrun))
                // } else if sr.timeout().bit_is_set() {
                //     panic!("p");
                //     // Err(nb::Error::Other(Error::Timeout))
                // } else if sr.berr().bit_is_set() {
                //     panic!("p");
                //     // Err(nb::Error::Other(Error::BusError))
                // } else if sr.tx_e().bit_is_set() {
                //     Ok(unsafe {
                //         ptr::write_volatile(&i2c.dr as *const _ as *mut u8, byte)
                //     })
                // } else {
                //     panic!("p");
                //     // Err(nb::Error::WouldBlock)
                // }
            }

        }
    }
}

impl_I2c!(I2C1);
// impl_I2c!(I2C2);
// impl_I2c!(I2C3);
