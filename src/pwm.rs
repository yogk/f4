//! Pulse Width Modulation
//!
//! You can use the `Pwm` interface with these TIM instances
//!
//! # TIM1
//!
//! - CH1 = PA8
//! - CH2 = PA9
//! - CH3 = PA10
//! - CH4 = PA11
//!
//! # TIM2
//!
//! - CH1 = PA0
//! - CH2 = PA1
//! - CH3 = PB10
//! - CH4 = PA3 (Unimplemented: conflicts with USB USART2_RX)
//!
//! # TIM3
//!
//! - CH1 = PA6
//! - CH2 = PC7
//! - CH3 = PB0
//! - CH4 = PB1
//!
//! # TIM4
//!
//! - CH1 = PB6
//! - CH2 = PB7
//! - CH3 = PB8
//! - CH4 = PB9

use core::any::{Any, TypeId};
// use core::marker::Unsize;

use cast::{u16, u32};
use hal;
// use static_ref::Static;
use stm32f40x::{DMA1, TIM1, TIM2, TIM3, TIM4, GPIOA, GPIOB, GPIOC, RCC};

// use dma::{self, Buffer, Dma1Channel2};
use timer::{Channel};

/// PWM driver
pub struct Pwm<'a, T>(pub &'a T)
where
    T: 'a;

macro_rules! impl_Pwm {
    ($TIM:ident, $APB:ident) => {
        impl<'a> Pwm<'a, $TIM>
        {
            /// Initializes the PWM module
            pub fn init<P>(
                &self,
                period: P,
                dma1: Option<&DMA1>,
                gpioa: &GPIOA,
                gpiob: &GPIOB,
                gpioc: &GPIOC,
                rcc: &RCC,
            ) where
                P: Into<::$APB::Ticks>,
            {
                self._init(period.into(), dma1, gpioa, gpiob, gpioc, rcc)
            }

            fn _init(
                &self,
                period: ::$APB::Ticks,
                dma1: Option<&DMA1>,
                gpioa: &GPIOA,
                gpiob: &GPIOB,
                gpioc: &GPIOC,
                rcc: &RCC,
            ) {
                let tim = self.0;

                // enable AFIO, (DMA1), TIMx and GPIOx
                if dma1.is_some() {
                    rcc.ahb1enr.modify(|_, w| w.dma1en().set_bit());
                }

                if tim.get_type_id() == TypeId::of::<TIM1>() {
                    rcc.apb2enr.modify(|_, w| w.tim1en().set_bit());
                } else if tim.get_type_id() == TypeId::of::<TIM2>() {
                    rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());
                } else if tim.get_type_id() == TypeId::of::<TIM3>() {
                    rcc.apb1enr.modify(|_, w| w.tim3en().set_bit());
                } else if tim.get_type_id() == TypeId::of::<TIM4>() {
                    rcc.apb1enr.modify(|_, w| w.tim4en().set_bit());
                }

                rcc.ahb1enr.modify(|_, w| {
                    if tim.get_type_id() == TypeId::of::<TIM1>() {
                        w.gpioaen().set_bit()
                    } else if tim.get_type_id() == TypeId::of::<TIM2>() {
                        w.gpioaen().set_bit().gpioben().set_bit()
                    } else if tim.get_type_id() == TypeId::of::<TIM3>() {
                        w.gpioaen().set_bit().gpioben().set_bit().gpiocen().set_bit()
                    } else if tim.get_type_id() == TypeId::of::<TIM4>() {
                        w.gpioben().set_bit()
                    } else {
                        unreachable!()
                    }
                });

                if tim.get_type_id() == TypeId::of::<TIM2>() {
                    // CH1 = PA0 = alternate push-pull
                    // CH2 = PA1 = alternate push-pull
                    // CH3 = PB10 = alternate push-pull
                    // CH4 = PA3 = alternate push-pull (Unimplemented: conflicts with USB USART2_RX)

                    // See datasheet DM00115249 Table 9. Alternate function mapping
                    gpioa.afrl.modify(|_, w| {
                        w
                        .afrl0().bits(1)
                        .afrl1().bits(1)
                    });
                    gpioa.moder.modify(|_, w| {
                        w
                        .moder0().bits(2)
                        .moder1().bits(2)
                    });
                    gpiob.afrh.modify(|_, w| { unsafe {
                        w
                        .afrh10().bits(1)}
                    });
                    gpiob.moder.modify(|_, w| { unsafe {
                        w
                        .moder10().bits(2)
                    }
                    });
                } else if tim.get_type_id() == TypeId::of::<TIM3>() {
                    // CH1 = PA6 = alternate push-pull
                    // CH2 = PC7 = alternate push-pull
                    // CH3 = PB0 = alternate push-pull
                    // CH4 = PB1 = alternate push-pull
                    gpioa.afrl.modify(|_, w| {
                        w.afrl6().bits(2)
                    });
                    gpioa.moder.modify(|_, w| {
                        w.moder6().bits(2)
                    });
                    gpiob.afrl.modify(|_, w| { unsafe {
                        w.afrl0().bits(2)
                        .afrl1().bits(2)
                        }
                    });
                    gpiob.moder.modify(|_, w| { unsafe {
                        w.moder0().bits(2)
                        .moder1().bits(2)
                        }
                    });
                    gpioc.afrl.modify(|_, w| { unsafe {
                        w.afrl7().bits(2)
                        }
                    });
                    gpioc.moder.modify(|_, w| { unsafe {
                        w.moder7().bits(2)
                        }
                    });    

                } else if tim.get_type_id() == TypeId::of::<TIM4>() {
                    // CH1 = PB6 = alternate push-pull
                    // CH2 = PB7 = alternate push-pull
                    // CH3 = PB8 = alternate push-pull
                    // CH4 = PB9 = alternate push-pull
                    gpiob.afrl.modify(|_, w| { unsafe {
                        w.afrl6().bits(2)
                        .afrl7().bits(2)
                        }
                    });
                    gpiob.moder.modify(|_, w| { unsafe {
                        w.moder6().bits(2)
                        .moder7().bits(2)
                        }
                    });
                    gpiob.afrh.modify(|_, w| { unsafe {
                        w.afrh8().bits(2)
                        .afrh9().bits(2)
                        }
                    });
                    gpiob.moder.modify(|_, w| { unsafe {
                        w.moder8().bits(2)
                        .moder9().bits(2)
                        }
                    });
                }

                // PWM mode 1
                if tim.get_type_id() == TypeId::of::<TIM2>() {
                    tim.ccmr1_output.modify(|_, w| unsafe {
                        w.oc1pe()
                            .set_bit()
                            .oc1m()
                            .bits(0b110)
                            .oc2pe()
                            .set_bit()
                            .oc2m()
                            .bits(0b110)
                    });
                    tim.ccmr2_output.modify(|_, w| unsafe {
                        w.oc3pe()
                            .set_bit()
                            .oc3m()
                            .bits(0b110)
                    });
                    tim.ccer
                        .modify(|_, w| w.cc1p().clear_bit().cc2p().clear_bit().cc3p().clear_bit());
                } else {
                    tim.ccmr1_output.modify(|_, w| unsafe {
                        w.oc1pe()
                            .set_bit()
                            .oc1m()
                            .bits(0b110)
                            .oc2pe()
                            .set_bit()
                            .oc2m()
                            .bits(0b110)
                    });

                    tim.ccmr2_output.modify(|_, w| unsafe {
                        w.oc3pe()
                            .set_bit()
                            .oc3m()
                            .bits(0b110)
                            .oc4pe()
                            .set_bit()
                            .oc4m()
                            .bits(0b110)
                    });

                    tim.ccer.modify(|_, w| {
                        w.cc1p()
                            .clear_bit()
                            .cc2p()
                            .clear_bit()
                            .cc3p()
                            .clear_bit()
                            .cc4p()
                            .clear_bit()
                    });
                }

                self._set_period(period);

                // if let Some(dma1) = dma1 {
                //     tim2.dier.modify(|_, w| w.ude().set_bit());

                //     if tim2.get_type_id() == TypeId::of::<TIM2>() {
                //         // TIM2_UP
                //         // mem2mem: Memory to memory mode disabled
                //         // pl: Medium priority
                //         // msize: Memory size = 8 bits
                //         // psize: Peripheral size = 16 bits
                //         // minc: Memory increment mode enabled
                //         // pinc: Peripheral increment mode disabled
                //         // circ: Circular mode disabled
                //         // dir: Transfer from memory to peripheral
                //         // tceie: Transfer complete interrupt enabled
                //         // en: Disabled
                //         dma1.ccr2.write(|w| unsafe {
                //             w.mem2mem()
                //                 .clear_bit()
                //                 .pl()
                //                 .bits(0b01)
                //                 .msize()
                //                 .bits(0b00)
                //                 .psize()
                //                 .bits(0b01)
                //                 .minc()
                //                 .set_bit()
                //                 .pinc()
                //                 .clear_bit()
                //                 .circ()
                //                 .clear_bit()
                //                 .dir()
                //                 .set_bit()
                //                 .tcie()
                //                 .set_bit()
                //                 .en()
                //                 .clear_bit()
                //         });
                //     } else {
                //         unimplemented!()
                //     }
                // }

                tim.cr1.write(|w| unsafe {
                    w.cms()
                        .bits(0b00)
                        .dir()
                        .bit(false)
                        .opm()
                        .bit(false)
                        .cen()
                        .set_bit()
                });
            }

            fn _set_period(&self, period: ::$APB::Ticks) {
                let period = period.0;

                let psc = u16((period - 1) / (1 << 16)).unwrap();
                self.0.psc.write(|w| unsafe{w.psc().bits(psc)});

                let arr = u32(period / u32(psc + 1));
                self.0.arr.write(|w| unsafe{w.bits(arr)});
            }

            // /// Uses `buffer` to continuously change the duty cycle on every period
            // pub fn set_duties<B>(
            //     &self,
            //     dma1: &DMA1,
            //     channel: Channel,
            //     buffer: &Static<Buffer<B, Dma1Channel2>>,
            // ) -> ::core::result::Result<(), dma::Error>
            // where
            //     B: Unsize<[u8]>,
            // {
            //     let tim2 = self.0;

            //     if tim2.get_type_id() == TypeId::of::<TIM2>() {
            //         if dma1.ccr2.read().en().bit_is_set() {
            //             return Err(dma::Error::InUse);
            //         }

            //         let buffer: &[u8] = buffer.lock();

            //         dma1.cndtr2
            //             .write(|w| unsafe { w.ndt().bits(u16(buffer.len()).unwrap()) });
            //         dma1.cpar2.write(|w| unsafe {
            //             match channel {
            //                 Channel::_1 => w.bits(&tim2.ccr1 as *const _ as u32),
            //                 Channel::_2 => w.bits(&tim2.ccr2 as *const _ as u32),
            //                 Channel::_3 => w.bits(&tim2.ccr3 as *const _ as u32),
            //                 Channel::_4 => w.bits(&tim2.ccr4 as *const _ as u32),
            //             }
            //         });
            //         dma1.cmar2
            //             .write(|w| unsafe { w.bits(buffer.as_ptr() as u32) });
            //         dma1.ccr2.modify(|_, w| w.en().set_bit());

            //         Ok(())

            //     } else {
            //         unimplemented!()
            //     }
            // }
        }

        impl<'a> hal::Pwm for Pwm<'a, $TIM>
        {
            type Channel = Channel;
            type Duty = u32;
            type Time = ::$APB::Ticks;

            fn get_duty(&self, channel: Channel) -> u32 {
                // if self.0.get_type_id() == TypeId::of::<TIM1>() {
                //     match channel {
                //         Channel::_1 => u32(u32(self.0.ccr1.read().bits()),
                //         Channel::_2 => u32(u32(self.0.ccr2.read().bits()),
                //         Channel::_3 => u32(u32(self.0.ccr3.read().bits()),
                //         Channel::_4 => u32(u32(self.0.ccr4.read().bits()),
                //     }
                // } else {
                    match channel {
                        Channel::_1 => u32(self.0.ccr1.read().ccr1_h().bits()) << 16 | u32(self.0.ccr1.read().ccr1_l().bits()),
                        Channel::_2 => u32(self.0.ccr2.read().ccr2_h().bits()) << 16 | u32(self.0.ccr2.read().ccr2_l().bits()),
                        Channel::_3 => u32(self.0.ccr3.read().ccr3_h().bits()) << 16 | u32(self.0.ccr3.read().ccr3_l().bits()),
                        Channel::_4 => u32(self.0.ccr4.read().ccr4_h().bits()) << 16 | u32(self.0.ccr4.read().ccr4_l().bits()),
                    }
                // }
            }

            fn disable(&self, channel: Channel) {
                match channel {
                    Channel::_1 => self.0.ccer.modify(|_, w| w.cc1e().clear_bit()),
                    Channel::_2 => self.0.ccer.modify(|_, w| w.cc2e().clear_bit()),
                    Channel::_3 => self.0.ccer.modify(|_, w| w.cc3e().clear_bit()),
                    Channel::_4 => self.0.ccer.modify(|_, w| w.cc4e().clear_bit()),
                }
            }

            fn enable(&self, channel: Channel) {
                match channel {
                    Channel::_1 => self.0.ccer.modify(|_, w| w.cc1e().set_bit()),
                    Channel::_2 => self.0.ccer.modify(|_, w| w.cc2e().set_bit()),
                    Channel::_3 => self.0.ccer.modify(|_, w| w.cc3e().set_bit()),
                    Channel::_4 => self.0.ccer.modify(|_, w| w.cc4e().set_bit()),
                }
            }

            fn get_max_duty(&self) -> u32 {
                self.0.arr.read().bits()
            }

            fn get_period(&self) -> ::$APB::Ticks {
                ::$APB::Ticks(u32(self.0.psc.read().bits() * self.0.arr.read().bits()))
            }

            fn set_duty(&self, channel: Channel, duty: u32) {
                let dutyl : u16 = u16(duty).unwrap();
                let dutyh : u16 = u16(duty >> 16).unwrap();
                match channel {
                    Channel::_1 => self.0.ccr1.write(|w| unsafe{w.ccr1_h().bits(dutyh).ccr1_l().bits(dutyl)}),
                    Channel::_2 => self.0.ccr2.write(|w| unsafe{w.ccr2_h().bits(dutyh).ccr2_l().bits(dutyl)}),
                    Channel::_3 => self.0.ccr3.write(|w| unsafe{w.ccr3_h().bits(dutyh).ccr3_l().bits(dutyl)}),
                    Channel::_4 => self.0.ccr4.write(|w| unsafe{w.ccr4_h().bits(dutyh).ccr4_l().bits(dutyl)}),
                }
            }

            fn set_period<P>(&self, period: P)
            where
                P: Into<::$APB::Ticks>,
            {
                self._set_period(period.into())
            }
        }
    }
}

// impl_Pwm!(TIM1,apb2);
impl_Pwm!(TIM2,apb1);
impl_Pwm!(TIM3,apb1);
impl_Pwm!(TIM4,apb1);