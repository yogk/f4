use core::mem;

use cast::{u16, u32};
use stm32f30x::TIM6;
use nb;

use hal;
use hal::Timer as _Timer;

use ahb;

// TODO make generic
pub struct Timer {
    tim: TIM6,
    timeout: ahb::Ticks,
}

impl Timer {
    pub fn new<T>(tim: TIM6, timeout: T) -> Timer
    where
        T: Into<ahb::Ticks>,
    {
        let mut timer = Timer {
            tim,
            timeout: unsafe { mem::uninitialized() },
        };

        timer.set_timeout(timeout);
        timer
    }

    pub fn release(self) -> TIM6 {
        self.tim
    }
}

impl hal::Timer for Timer {
    type Time = ahb::Ticks;

    fn get_timeout(&self) -> Self::Time {
        self.timeout
    }

    fn pause(&mut self) {
        self.tim.cr1.modify(|_, w| w.cen().clear_bit());
    }

    fn restart(&mut self) {
        self.tim.cnt.write(|w| unsafe { w.cnt().bits(0) });
    }

    fn resume(&mut self) {
        self.tim.cr1.modify(|_, w| w.cen().set_bit());
    }

    fn set_timeout<T>(&mut self, timeout: T)
    where
        T: Into<Self::Time>,
    {
        let period = timeout.into().0;

        let psc = u16((period - 1) / (1 << 16)).unwrap();
        self.tim.psc.write(|w| w.psc().bits(psc));

        let arr = u16(period / u32(psc + 1)).unwrap();
        self.tim.arr.write(|w| w.arr().bits(arr));
    }

    fn wait(&mut self) -> nb::Result<(), !> {
        if self.tim.sr.read().uif().bit_is_clear() {
            Err(nb::Error::WouldBlock)
        } else {
            self.tim.sr.modify(|_, w| w.uif().clear_bit());
            Ok(())
        }
    }
}
