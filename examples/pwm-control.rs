//! Output a PWM on pin PA0 and control its duty cycle via a serial interface
//!
//! - '*' increase duty by a factor of 2
//! - '+' increase duty by 1
//! - '-' decrease duty by 1
//! - '/' decrease duty by a factor of 2
#![deny(unsafe_code)]
#![deny(warnings)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m_rtfm as rtfm;
extern crate f4;

use core::u32;

use f4::prelude::*;
use f4::time::Hertz;
use f4::{Channel, Pwm, Serial};
use f4::serial::Event;
use rtfm::{app, Threshold};

const BAUD_RATE: Hertz = Hertz(115_200);
const FREQUENCY: Hertz = Hertz(1_000);

app! {
    device: f4::stm32f40x,

    tasks: {
        USART2: {
            path: rx,
            resources: [TIM4, USART2],
        },
    },
}

fn init(p: init::Peripherals) {
    let pwm = Pwm(p.TIM4);
    let serial = Serial(p.USART2);

    serial.init(BAUD_RATE.invert(), None, p.GPIOA, p.RCC);
    serial.listen(Event::Rxne);

    pwm.init(FREQUENCY.invert(), None, p.GPIOA, p.GPIOB, p.GPIOC, p.RCC);
    pwm.set_duty(Channel::_1, 0);

    pwm.enable(Channel::_1);
}

fn idle() -> ! {
    loop {
        rtfm::wfi();
    }
}

fn rx(_t: &mut Threshold, r: USART2::Resources) {
    let pwm = Pwm(&**r.TIM4);
    let serial = Serial(&**r.USART2);

    let byte = serial.read().unwrap();
    // Echo back to signal we are alive
    serial.write(byte).unwrap();

    match byte {
        b'+' | b'-' | b'*' | b'/' => {
            let duty = pwm.get_duty(Channel::_1);

            match byte {
                b'+' => {
                    let max = pwm.get_max_duty();
                    pwm.set_duty(Channel::_1, if duty < max { duty + 1 } else { max });
                }
                b'-' => {
                    pwm.set_duty(Channel::_1, duty.checked_sub(1).unwrap_or(0));
                }
                b'*' => {
                    let new_duty = duty.checked_mul(2).unwrap_or(u32::MAX);
                    let max_duty = pwm.get_max_duty();

                    if new_duty < max_duty {
                        pwm.set_duty(Channel::_1, new_duty)
                    }
                }
                b'/' => pwm.set_duty(Channel::_1, duty / 2),
                _ => { /* unreachable */ }
            }
        }
        _ => {}
    }
}
