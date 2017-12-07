//! Timer

use core::any::{Any, TypeId};

use cast::{u16, u32};
use hal;
use nb::{self, Error};
use stm32f40x::{TIM1, TIM2, TIM3, TIM4, TIM5, TIM9, TIM10, TIM11, RCC};

/// Channel associated to a timer
#[derive(Clone, Copy, Debug)]
pub enum Channel {
    /// TxC1
    _1,
    /// TxC2
    _2,
    /// TxC3
    _3,
    /// TxC4
    _4,
}


/// `hal::Timer` implementation
pub struct Timer<'a, T>(pub &'a T)
where
    T: 'a;

impl<'a, T> Clone for Timer<'a, T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<'a, T> Copy for Timer<'a, T> {}

macro_rules! impl_Timer {
    ($TIM:ident, $APB:ident) => {
        impl<'a> Timer<'a, $TIM>
        {
            /// Initializes the timer with a periodic timeout of `frequency` Hz
            ///
            /// NOTE After initialization, the timer will be in the paused state.
            pub fn init<P>(&self, period: P, rcc: &RCC)
            where
                P: Into<::$APB::Ticks>,
            {
                self.init_(period.into(), rcc)
            }

            fn init_(&self, timeout: ::$APB::Ticks, rcc: &RCC) {
                let tim = self.0;

                // Enable TIMx
                // Reference manual does not mention TIM6-8, TIM 12-14 
                // although they are implemented in device crate...
                if tim.get_type_id() == TypeId::of::<TIM1>() {
                    rcc.apb2enr.modify(|_, w| w.tim1en().set_bit());
                } else if tim.get_type_id() == TypeId::of::<TIM2>() {
                    rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());
                } else if tim.get_type_id() == TypeId::of::<TIM3>() {
                    rcc.apb1enr.modify(|_, w| w.tim3en().set_bit());
                } else if tim.get_type_id() == TypeId::of::<TIM4>() {
                    rcc.apb1enr.modify(|_, w| w.tim4en().set_bit());
                } else if tim.get_type_id() == TypeId::of::<TIM5>() {
                    rcc.apb1enr.modify(|_, w| w.tim5en().set_bit());
                } else if tim.get_type_id() == TypeId::of::<TIM9>() {
                    rcc.apb2enr.modify(|_, w| w.tim9en().set_bit());
                } else if tim.get_type_id() == TypeId::of::<TIM10>() {
                    rcc.apb2enr.modify(|_, w| w.tim10en().set_bit());
                } else if tim.get_type_id() == TypeId::of::<TIM11>() {
                    rcc.apb2enr.modify(|_, w| w.tim11en().set_bit());
                }

                // Configure periodic update event
                self._set_timeout(timeout);

                // Continuous mode
                // tim2.cr1.write(|w| w.opm().continuous());

                // Enable the update event interrupt
                tim.dier.modify(|_, w| w.uie().set_bit());
            }

            fn _set_timeout(&self, timeout: ::$APB::Ticks) {
                let period = timeout.0;

                let psc = u16((period - 1) / (1 << 16)).unwrap();
                self.0.psc.write(|w| unsafe{w.psc().bits(psc)});

                let arr = u32(period / u32(psc + 1));
                self.0.arr.write(|w| unsafe{w.bits(arr)});
            }
        }

        impl<'a> hal::Timer for Timer<'a, $TIM>
            {
            type Time = ::$APB::Ticks;

            fn get_timeout(&self) -> ::$APB::Ticks {
                    ::$APB::Ticks(
                        u32(self.0.psc.read().psc().bits() + 1) *
                            u32(self.0.arr.read().bits()),
                    )
                }

                fn pause(&self) {
                    self.0.cr1.modify(|_, w| w.cen().clear_bit());
                }

                fn restart(&self) {
                    self.0.cnt.write(|w| unsafe{w.bits(0)});
                }

                fn resume(&self) {
                    self.0.cr1.modify(|_, w| w.cen().set_bit());
                }

                fn set_timeout<TO>(&self, timeout: TO)
                where
                    TO: Into<::$APB::Ticks>,
                {
                    self._set_timeout(timeout.into())
                }

                fn wait(&self) -> nb::Result<(), !> {
                    if self.0.sr.read().uif().bit_is_clear() {
                        Err(Error::WouldBlock)
                    } else {
                        self.0.sr.modify(|_, w| w.uif().clear_bit());
                        Ok(())
                    }
                }
            }
    }
}

impl_Timer!(TIM1, apb2);
impl_Timer!(TIM2, apb1);
impl_Timer!(TIM3, apb1);
impl_Timer!(TIM4, apb1);
impl_Timer!(TIM5, apb1);
impl_Timer!(TIM9, apb2);
impl_Timer!(TIM10, apb2);
impl_Timer!(TIM11, apb2);
