//! Input capture interface
//!
//! You can use the `Capture` interface with these TIM instances:
//!
//! # TIM1
//!
//! - CH1 = PA8 (5V tolerant)
//! - CH2 = PA9 (5V tolerant)
//! - CH3 = PA10 (5V tolerant)
//! - CH4 = PA11 (5V tolerant)
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
//! - CH2 = PA7
//! - CH3 = PB0
//! - CH4 = PB1
//!
//! **WARNING** Do not use channels 3 and 4 with the `Capture.capture` API or
//! you'll get junk values.
//!
//! # TIM4
//!
//! - CH1 = PB6 (5V tolerant)
//! - CH2 = PB7 (5V tolerant)
//! - CH3 = PB8 (5V tolerant)
//! - CH4 = PB9 (5V tolerant)

use core::any::{Any, TypeId};
use core::u32;

use cast::u32;
use hal;
use nb;
use stm32f40x::{TIM1, TIM2, TIM3, TIM4, GPIOA, GPIOB, GPIOC, RCC};

use timer::Channel;

/// Input / capture error
#[derive(Debug)]
pub enum Error {
    /// Previous capture value was overwritten
    Overcapture,
    #[doc(hidden)] _Extensible,
}

/// Interrupt event
pub enum Event {
    /// Capture on channel 1
    Capture1,
    /// Capture on channel 2
    Capture2,
    /// Capture on channel 3
    Capture3,
    /// Capture on channel 4
    Capture4,
}

/// Input capture interface
pub struct Capture<'a, T>(pub &'a T)
where
    T: 'a;

impl<'a, T> Clone for Capture<'a, T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<'a, T> Copy for Capture<'a, T> {}

macro_rules! impl_Capture {
    ($TIM:ident, $APB:ident) => {
        impl<'a> Capture<'a, $TIM>
        {
            /// Initializes the input capture interface
            ///
            /// `resolution` is the resolution of the capture timer
            pub fn init<R>(
                &self,
                resolution: R,
                channel: Channel,
                gpioa: &GPIOA, // TODO: Make these optional/implement custom init for each TIM
                gpiob: &GPIOB,
                gpioc: &GPIOC,
                rcc: &RCC)
            where
                R: Into<::$APB::Ticks>,
            {
                self._init(resolution.into(), channel, gpioa, gpiob, gpioc, rcc)
            }

            fn _init(
                &self,
                resolution: ::$APB::Ticks,
                channel: Channel,
                gpioa: &GPIOA,
                gpiob: &GPIOB,
                gpioc: &GPIOC,
                rcc: &RCC) {
                let tim = self.0;

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
                        match channel {
                            Channel::_1 =>  w.gpioaen().set_bit(),
                            Channel::_2 =>  w.gpioaen().set_bit(),
                            Channel::_3 =>  w.gpioben().set_bit(),
                            Channel::_4 =>  panic!("Not implemented: conflicts with USB USART2_RX"),
                        }
                    } else if tim.get_type_id() == TypeId::of::<TIM3>() {
                        match channel {
                            Channel::_1 =>  w.gpioaen().set_bit(),
                            Channel::_2 =>  w.gpiocen().set_bit(),
                            Channel::_3 =>  w.gpioben().set_bit(),
                            Channel::_4 =>  w.gpioben().set_bit(),
                        }
                    } else if tim.get_type_id() == TypeId::of::<TIM4>() {
                        w.gpioben().set_bit()
                    } else {
                        unreachable!()
                    }
                });

                // See datasheet DM00115249 Table 9. Alternate function mapping
                if tim.get_type_id() == TypeId::of::<TIM1>() {
                    // CH1 = PA8 = alternate push-pull
                    // CH2 = PA9 = alternate push-pull
                    // CH3 = PA10 = alternate push-pull
                    // CH4 = PA11 = alternate push-pull
                    match channel {
                        Channel::_1 => {
                            gpioa.afrh.modify(|_, w|  w.afrh8().bits(1));
                            gpioa.moder.modify(|_, w| w.moder8().bits(2));
                        }
                        Channel::_2 => {
                            gpioa.afrh.modify(|_, w|  w.afrh9().bits(1));
                            gpioa.moder.modify(|_, w| w.moder9().bits(2));
                        }
                        Channel::_3 => {
                            gpioa.afrh.modify(|_, w|  w.afrh10().bits(1));
                            gpioa.moder.modify(|_, w| w.moder10().bits(2));
                        }
                        Channel::_4 => {
                            gpioa.afrh.modify(|_, w|  w.afrh11().bits(1));
                            gpioa.moder.modify(|_, w| w.moder11().bits(2));
                        }
                    }
                } else if tim.get_type_id() == TypeId::of::<TIM2>() {
                    // CH1 = PA0 = alternate push-pull
                    // CH2 = PA1 = alternate push-pull
                    // CH3 = PB10 = alternate push-pull
                    // CH4 = PA3 = alternate push-pull (Not implemented: conflicts with USB USART2_RX)

                    // See datasheet DM00115249 Table 9. Alternate function mapping
                    match channel {
                        Channel::_1 => {
                            gpioa.afrl.modify(|_, w| w.afrl0().bits(1));
                            gpioa.moder.modify(|_, w| w.moder0().bits(2));
                        }
                        Channel::_2 => {
                            gpioa.afrl.modify(|_, w| w.afrl1().bits(1));
                            gpioa.moder.modify(|_, w| w.moder1().bits(2));
                        }
                        Channel::_3 => {
                            gpiob.afrh.modify(|_, w|  unsafe {w.afrh10().bits(1)});
                            gpiob.moder.modify(|_, w| unsafe {w.moder10().bits(2)});
                        }
                        Channel::_4 => {
                            panic!("Not implemented: conflicts with USB USART2_RX");
                        }
                    }
                } else if tim.get_type_id() == TypeId::of::<TIM3>() {
                    // CH1 = PA6 = alternate push-pull
                    // CH2 = PC7 = alternate push-pull
                    // CH3 = PB0 = alternate push-pull
                    // CH4 = PB1 = alternate push-pull
                    match channel {
                        Channel::_1 => {
                            gpioa.afrl.modify(|_, w| w.afrl6().bits(2));
                            gpioa.moder.modify(|_, w| w.moder6().bits(2));
                        }
                        Channel::_2 => {
                            gpioc.afrl.modify(|_, w|  unsafe {w.afrl7().bits(2)});
                            gpioc.moder.modify(|_, w| unsafe {w.moder7().bits(2)});
                        }
                        Channel::_3 => {
                            gpiob.afrl.modify(|_, w|  unsafe {w.afrl0().bits(2)});
                            gpiob.moder.modify(|_, w| unsafe {w.moder0().bits(2)});
                        }
                        Channel::_4 => {
                            gpiob.afrl.modify(|_, w|  unsafe {w.afrl1().bits(2)});
                            gpiob.moder.modify(|_, w| unsafe {w.moder1().bits(2)});
                        }
                    }

                } else if tim.get_type_id() == TypeId::of::<TIM4>() {
                    // CH1 = PB6 = alternate push-pull
                    // CH2 = PB7 = alternate push-pull
                    // CH3 = PB8 = alternate push-pull
                    // CH4 = PB9 = alternate push-pull
                    match channel {
                        Channel::_1 => {
                            gpiob.afrl.modify(|_, w|  unsafe {w.afrl6().bits(2)});
                            gpiob.moder.modify(|_, w| unsafe {w.moder6().bits(2)});
                        }
                        Channel::_2 => {
                            gpiob.afrl.modify(|_, w|  unsafe {w.afrl7().bits(2)});
                            gpiob.moder.modify(|_, w| unsafe {w.moder7().bits(2)});
                        }
                        Channel::_3 => {
                            gpiob.afrh.modify(|_, w|  unsafe {w.afrh8().bits(2)});
                            gpiob.moder.modify(|_, w| unsafe {w.moder8().bits(2)});
                        }
                        Channel::_4 => {
                            gpiob.afrh.modify(|_, w|  unsafe {w.afrh9().bits(2)});
                            gpiob.moder.modify(|_, w| unsafe {w.moder9().bits(2)});
                        }
                    }
                }

                tim.smcr.write(|w| unsafe {
                    w.bits(0)
                });
                // configure CC{1,2,3,4} as input and wire it to TI{1,2,3,4}
                // apply the heaviest filter
                tim.ccmr1_output.write(|w| unsafe {
                    w.bits((0b1111 << 12) | (0b01 << 8) | (0b1111 << 4) | (0b01 << 0))
                });
                tim.ccmr2_output.write(|w| unsafe {
                    w.bits((0b1111 << 12) | (0b01 << 8) | (0b1111 << 4) | (0b01 << 0))
                });

                // enable capture on rising edge
                // capture pins disabled by default
                match channel {
                    Channel::_1 => {
                        tim.ccer.modify(|_, w| {w.cc1p().clear_bit().cc1e().clear_bit()});
                    }
                    Channel::_2 => {
                        tim.ccer.modify(|_, w| {w.cc2p().clear_bit().cc2e().clear_bit()});
                    }
                    Channel::_3 => {
                        tim.ccer.modify(|_, w| {w.cc3p().clear_bit().cc3e().clear_bit()});
                    }
                    Channel::_4 => {
                        if tim.get_type_id() == TypeId::of::<TIM2>() {
                            panic!("Not implemented: conflicts with USB USART2_RX");
                        }
                        tim.ccer.modify(|_, w| {w.cc4p().clear_bit().cc4e().clear_bit()});
                    }
                }

                self._set_resolution(resolution);

                tim.arr.write(|w| unsafe{ w.bits(u32::MAX) });

                // RM0368 13.4.1
                // udis: Update event disabled, shadow registers keep their value (ARR, PSC, CCRx)
                // dir: Upcounter
                // opm: One-pulse mode disabled
                // cen: Counter enabled
                tim.cr1.write(|w| w.udis().set_bit().dir().bit(false).opm().bit(false).cen().set_bit());
            }

            /// Starts listening for an interrupt `event`
            pub fn listen(&self, event: Event) {
                let tim = self.0;

                match event {
                    Event::Capture1 => tim.dier.modify(|_, w| w.cc1ie().set_bit()),
                    Event::Capture2 => tim.dier.modify(|_, w| w.cc2ie().set_bit()),
                    Event::Capture3 => tim.dier.modify(|_, w| w.cc3ie().set_bit()),
                    Event::Capture4 => tim.dier.modify(|_, w| w.cc4ie().set_bit()),
                }
            }

            /// Stops listening for an interrupt `event`
            pub fn unlisten(&self, event: Event) {
                let tim = self.0;

                match event {
                    Event::Capture1 => tim.dier.modify(|_, w| w.cc1ie().clear_bit()),
                    Event::Capture2 => tim.dier.modify(|_, w| w.cc2ie().clear_bit()),
                    Event::Capture3 => tim.dier.modify(|_, w| w.cc3ie().clear_bit()),
                    Event::Capture4 => tim.dier.modify(|_, w| w.cc4ie().clear_bit()),
                }
            }

            fn _set_resolution(&self, resolution: ::$APB::Ticks) {
                let psc = resolution.0.checked_sub(1).expect("impossible resolution");

                self.0.psc.write(|w| unsafe{ w.bits(psc)});
            }

            /// Clear the overcapture bit of channel
            pub fn clear(&self, channel: Channel ) {
                match channel {
                    Channel::_1 => {
                        self.0.ccr1.read().bits();
                        self.0.sr.modify(|_,w|  w.cc1of().clear_bit() )},
                    Channel::_2 => {
                        self.0.ccr2.read().bits();
                        self.0.sr.modify(|_,w|  w.cc2of().clear_bit() )},
                    Channel::_3 => {
                        self.0.ccr3.read().bits();
                        self.0.sr.modify(|_,w|  w.cc3of().clear_bit() )},
                    Channel::_4 => {
                        self.0.ccr4.read().bits();
                        self.0.sr.modify(|_,w|  w.cc4of().clear_bit() )},
                }
            }
        }

        impl<'a> hal::Capture for Capture<'a, $TIM>
        {
            type Capture = u32;
            type Channel = Channel;
            type Error = Error;
            type Time = ::$APB::Ticks;

            fn capture(&self, channel: Channel) -> nb::Result<u32, Error> {
                let tim = self.0;
                let sr = tim.sr.read();

                match channel {
                    Channel::_1 => if sr.cc1of().bit_is_set() {
                        Err(nb::Error::Other(Error::Overcapture))
                    } else if sr.cc1if().bit_is_set() {
                        Ok(tim.ccr1.read().bits())
                    } else {
                        Err(nb::Error::WouldBlock)
                    },
                    Channel::_2 => if sr.cc2of().bit_is_set() {
                        Err(nb::Error::Other(Error::Overcapture))
                    } else if sr.cc2if().bit_is_set() {
                        Ok(tim.ccr2.read().bits())
                    } else {
                        Err(nb::Error::WouldBlock)
                    },
                    Channel::_3 => if sr.cc3of().bit_is_set() {
                        Err(nb::Error::Other(Error::Overcapture))
                    } else if sr.cc3if().bit_is_set() {
                        Ok(tim.ccr3.read().bits())
                    } else {
                        Err(nb::Error::WouldBlock)
                    },
                    Channel::_4 => if sr.cc4of().bit_is_set() {
                        Err(nb::Error::Other(Error::Overcapture))
                    } else if sr.cc4if().bit_is_set() {
                        Ok(tim.ccr4.read().bits())
                    } else {
                        Err(nb::Error::WouldBlock)
                    },
                }
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

            fn get_resolution(&self) -> ::$APB::Ticks {
                ::$APB::Ticks(u32(self.0.psc.read().psc().bits()))
            }

            fn set_resolution<R>(&self, resolution: R)
            where
                R: Into<::$APB::Ticks>,
            {
                self._set_resolution(resolution.into())
            }
        }
    }
}

impl_Capture!(TIM1, apb2);
impl_Capture!(TIM2, apb1);
impl_Capture!(TIM3, apb1);
impl_Capture!(TIM4, apb1);
