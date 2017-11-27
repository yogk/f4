//! Periodic timer

use core::u16;

use cast::{u16, u32};
use stm32f40x::{RCC, TIM11};

/// Specialized `Result` type
pub type Result<T> = ::core::result::Result<T, Error>;

/// An error
pub struct Error {
    _0: (),
}

/// Periodic timer
///
/// # Interrupts
///
/// - `Tim11` - update event
pub struct Timer<'a>(pub &'a TIM11);

impl<'a> Timer<'a> {
    /// Initializes the timer with a periodic timeout of `frequency` Hz
    ///
    /// NOTE After initialization, the timer will be in the paused state.
    pub fn init(&self, rcc: &RCC, frequency: u32) {
        let tim11 = self.0;

        // Power up peripherals
        rcc.apb2enr.modify(|_, w| w.tim11en().set_bit());

        let ratio = ::apb1::FREQUENCY / frequency;
        let psc = u16((ratio - 1) / u32(u16::MAX)).unwrap();
        tim11.psc.write(|w| unsafe{w.psc().bits(psc)});
        let arr = u16(ratio / u32(psc + 1)).unwrap();
        tim11.arr.write(|w| unsafe{ w.arr().bits(arr)});

        tim11.dier.write(|w| w.uie().set_bit());
        // tim7.cr1.write(|w| w.opm().continuous());
    }

    /// Clears the update event flag
    ///
    /// Returns `Err` if no update event has occurred
    pub fn clear_update_flag(&self) -> Result<()> {
        let tim11 = self.0;

        if tim11.sr.read().uif().bit_is_clear() {
            Err(Error { _0: () })
        } else {
            self.0.sr.modify(|_, w| w.uif().clear_bit());
            Ok(())
        }
    }

    /// Resumes the timer count
    pub fn resume(&self) {
        self.0.cr1.modify(|_, w| w.cen().set_bit());
    }

    /// Pauses the timer
    pub fn pause(&self) {
        self.0.cr1.modify(|_, w| w.cen().clear_bit());
    }
}
