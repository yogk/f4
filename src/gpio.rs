use core::ptr;

use stm32f30x::{GPIOA, GPIOB, GPIOC, GPIOD, GPIOE};

pub trait GpioExt {
    type Pins;

    fn pins(self) -> Self::Pins;
}

macro_rules! structs {
    ($($struct:ident,)+) => {
        $(
            pub struct $struct<P> {
                pin: P,
            }

            impl<P> $struct<P> {
                pub fn release(self) -> P {
                    self.pin
                }
            }
        )+
    }
}

structs!(
    Af0,
    Af1,
    Af2,
    Af3,
    Af4,
    Af5,
    Af6,
    Af7,
    Af8,
    Af9,
    Af10,
    Af11,
    Af12,
    Af13,
    Af14,
    Af15,
    Input,
    Output,
);

macro_rules! pins {
    ($GPIO:ident, $gpio:ident, [$(($pin:ident, $n:expr, $moder:ident)),+]) => {
        pub mod $gpio {
            #[allow(non_snake_case)]
            pub struct Pins {
                $(pub $pin: super::$pin,)+
                pub MODER: MODER,
                pub AFRL: AFRL,
                pub AFRH: AFRH,
            }

            pub struct AFRL { _0: () }

            impl AFRL {
                #[doc(hidden)]
                pub unsafe fn new() -> Self {
                    AFRL { _0: () }
                }
            }

            pub struct AFRH { _0: () }

            impl AFRH {
                #[doc(hidden)]
                pub unsafe fn new() -> Self {
                    AFRH { _0: () }
                }
            }

            pub struct MODER { _0: () }

            impl MODER {
                #[doc(hidden)]
                pub unsafe fn new() -> Self {
                    MODER { _0: () }
                }
            }
        }

        impl GpioExt for $GPIO {
            type Pins = $gpio::Pins;

            fn pins(self) -> $gpio::Pins {
                $gpio::Pins {
                    $($pin: $pin { _0: () } ,)+
                    AFRL: unsafe { $gpio::AFRL::new() },
                    AFRH: unsafe { $gpio::AFRH::new() },
                    MODER: unsafe { $gpio::MODER::new() },
                }
            }
        }

        $(
            pub struct $pin {
                _0: (),
            }

            // FIXME bit banding is not possible
            impl $pin {
                pub fn as_input(self, _moder: &mut $gpio::MODER) -> Input<$pin> {
                    unsafe {
                        (*$GPIO::ptr()).moder.modify(|_, w| w.$moder().input());
                    }

                    Input { pin: self }
                }

                pub fn as_output(self, _moder: &mut $gpio::MODER) -> Output<$pin> {
                    unsafe {
                        (*$GPIO::ptr()).moder.modify(|_, w| w.$moder().output());
                    }

                    Output { pin: self }
                }
            }

            impl Input<$pin> {
                pub fn is_high(&self) -> bool {
                    let mask = 1 << $n;
                    unsafe {
                        ((*$GPIO::ptr()).idr.read().bits() & mask) != 0
                    }
                }

                pub fn is_low(&self) -> bool {
                    !self.is_high()
                }
            }

            impl Output<$pin> {
                /// Sets the pin high
                pub fn high(&mut self) {
                    unsafe {
                        ptr::write_volatile(
                            &(*$GPIO::ptr()).bsrr as *const _ as *mut u32,
                            1 << $n
                        )
                    }
                }

                /// Sets the pin high
                pub fn low(&mut self) {
                    unsafe {
                        ptr::write_volatile(
                            &(*$GPIO::ptr()).bsrr as *const _ as *mut u32,
                            1 << (16 + $n)
                        )
                    }
                }
            }
        )+
    }
}

pins!(
    GPIOA,
    gpioa,
    [
        (PA0, 0, moder0),
        (PA1, 1, moder1),
        (PA2, 2, moder2),
        (PA3, 3, moder3),
        (PA4, 4, moder4),
        (PA5, 5, moder5),
        (PA6, 6, moder6),
        (PA7, 7, moder7),
        (PA8, 8, moder8),
        (PA9, 9, moder9),
        (PA10, 10, moder10),
        (PA11, 11, moder11),
        (PA12, 12, moder12),
        (PA13, 13, moder13),
        (PA14, 14, moder14),
        (PA15, 15, moder15)
    ]
);

pins!(
    GPIOB,
    gpiob,
    [
        (PB0, 0, moder0),
        (PB1, 1, moder1),
        (PB2, 2, moder2),
        (PB3, 3, moder3),
        (PB4, 4, moder4),
        (PB5, 5, moder5),
        (PB6, 6, moder6),
        (PB7, 7, moder7),
        (PB8, 8, moder8),
        (PB9, 9, moder9),
        (PB10, 10, moder10),
        (PB11, 11, moder11),
        (PB12, 12, moder12),
        (PB13, 13, moder13),
        (PB14, 14, moder14),
        (PB15, 15, moder15)
    ]
);

pins!(
    GPIOC,
    gpioc,
    [
        (PC0, 0, moder0),
        (PC1, 1, moder1),
        (PC2, 2, moder2),
        (PC3, 3, moder3),
        (PC4, 4, moder4),
        (PC5, 5, moder5),
        (PC6, 6, moder6),
        (PC7, 7, moder7),
        (PC8, 8, moder8),
        (PC9, 9, moder9),
        (PC10, 10, moder10),
        (PC11, 11, moder11),
        (PC12, 12, moder12),
        (PC13, 13, moder13),
        (PC14, 14, moder14),
        (PC15, 15, moder15)
    ]
);

pins!(
    GPIOD,
    gpiod,
    [
        (PD0, 0, moder0),
        (PD1, 1, moder1),
        (PD2, 2, moder2),
        (PD3, 3, moder3),
        (PD4, 4, moder4),
        (PD5, 5, moder5),
        (PD6, 6, moder6),
        (PD7, 7, moder7),
        (PD8, 8, moder8),
        (PD9, 9, moder9),
        (PD10, 10, moder10),
        (PD11, 11, moder11),
        (PD12, 12, moder12),
        (PD13, 13, moder13),
        (PD14, 14, moder14),
        (PD15, 15, moder15)
    ]
);

pins!(
    GPIOE,
    gpioe,
    [
        (PE0, 0, moder0),
        (PE1, 1, moder1),
        (PE2, 2, moder2),
        (PE3, 3, moder3),
        (PE4, 4, moder4),
        (PE5, 5, moder5),
        (PE6, 6, moder6),
        (PE7, 7, moder7),
        (PE8, 8, moder8),
        (PE9, 9, moder9),
        (PE10, 10, moder10),
        (PE11, 11, moder11),
        (PE12, 12, moder12),
        (PE13, 13, moder13),
        (PE14, 14, moder14),
        (PE15, 15, moder15)
    ]
);

impl PA9 {
    pub fn af7(self, _afrl: &mut gpioa::AFRH, _moder: &mut gpioa::MODER) -> Af7<PA9> {
        unsafe {
            (*GPIOA::ptr()).moder.modify(|_, w| w.moder9().alternate());
            (*GPIOA::ptr()).afrh.modify(|_, w| w.afrh9().bits(7));
        }

        Af7 { pin: self }
    }
}

impl PA10 {
    pub fn af7(self, _afrl: &mut gpioa::AFRH, _moder: &mut gpioa::MODER) -> Af7<PA10> {
        unsafe {
            (*GPIOA::ptr()).moder.modify(|_, w| w.moder10().alternate());
            (*GPIOA::ptr()).afrh.modify(|_, w| w.afrh10().bits(7));
        }

        Af7 { pin: self }
    }
}
