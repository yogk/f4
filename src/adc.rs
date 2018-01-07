//! Analog to Digital Converter

use core::marker::Unsize;

use cast::u16;
use hal::prelude::*;
use static_ref::Static;

use dma::{self, CircBuffer, Dma2Stream0};
use stm32f40x::{ADC1, DMA2, TIM2, GPIOA, RCC};
use {Channel, Pwm};

/// Input associated to ADC1
#[derive(Clone, Copy, Debug)]
pub enum AdcIn {
    /// ADC1_IN0
    _0,
    /// ADC1_IN1
    _1,
    /// ADC1_IN2
    _2,
    /// ADC1_IN3
    _3,
    /// ADC1_IN4
    _4,
    /// ADC1_IN5
    _5,
    /// ADC1_IN6
    _6,
    /// ADC1_IN7
    _7,
    /// ADC1_IN8
    _8,
    /// ADC1_IN9
    _9,
    /// ADC1_IN10
    _10,
    /// ADC1_IN11
    _11,
    /// ADC1_IN12
    _12,
    /// ADC1_IN13
    _13,
    /// ADC1_IN14
    _14,
    /// ADC1_IN15
    _15,
}

/// ADC1
pub struct Adc<'a>(pub &'a ADC1);

impl<'a> Adc<'a> {
    /// Initializes the ADC
    ///
    /// NOTE `Pwm<TIM2>.init` must be called before this method because both
    /// methods configure the PA1 pin (one as input and the other as output :-/)
    pub fn init(&self, dma2: &DMA2, gpioa: &GPIOA, rcc: &RCC) {
        let adc1 = self.0;

        // enable ADC1, DMA1, GPIOA, TIM2
        rcc.ahb1enr.modify(|_, w| w.dma2en().set_bit());
        rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());
        rcc.apb2enr.modify(|_, w| w.adc1en().set_bit());

        // Set PA1 as analog input
        gpioa.afrl.modify(|_, w| w.afrl1().bits(0));
        gpioa.moder.modify(|_, w| w.moder1().bits(0b11));
        gpioa.pupdr.modify(|_, w| unsafe { w.pupdr1().bits(0b00) });

        // Sample only the channel 1
        adc1.sqr1.modify(|_, w| unsafe { w.l().bits(1) });
        adc1.sqr3.modify(|_, w| unsafe { w.sq1().bits(1) });

        // RM0368 11.12.5
        // Sample time: 55.5 + 12.5 = 68 cycles
        adc1.smpr2.modify(|_, w| unsafe { w.smpx_x().bits(0) });

        // ADC1
        // chsel: Channel 0 (RM0368 9.3.3 Table 27)
        // pl: Medium priority
        // msize: Memory size = 16 bits
        // psize: Peripheral size = 16 bits
        // minc: Memory increment mode enabled
        // pinc: Peripheral increment mode disabled
        // circ: Circular mode enabled
        // dir: Transfer from peripheral to memory
        // htie: Half transfer interrupt enabled
        // tceie: Transfer complete interrupt enabled
        // en: Disabled
        dma2.s0cr.write(|w| unsafe {
            w.chsel()
                .bits(0)
                .pl()
                .bits(0b01)
                .msize()
                .bits(0b01)
                .psize()
                .bits(0b01)
                .minc()
                .set_bit()
                .circ()
                .set_bit()
                .pinc()
                .clear_bit()
                .dir()
                .bits(0)
                .htie()
                .set_bit()
                .tcie()
                .set_bit()
                .en()
                .clear_bit()
        });

        // exten: Conversion on external trigger rising edge
        // extsel: Timer 2 CC2 event
        // align: Right alignment
        // dma: DMA mode enabled
        // cont: Single conversion mode
        // adon: Disable ADC conversion
        adc1.cr2.write(|w| unsafe {
            w.exten()
                .bits(0b01)
                .extsel()
                .bits(0b011) // T2C2
                .align()
                .clear_bit()
                .dma()
                .set_bit()
                .dds()
                .set_bit()
                .cont()
                .clear_bit()
                .adon()
                .clear_bit()
        });
    }

    /// Disables the ADC
    pub fn disable(&self) {
        self.0.cr2.modify(|_, w| w.adon().clear_bit());
    }

    /// Enables the ADC
    pub fn enable(&self) {
        self.0.cr2.modify(|_, w| w.adon().set_bit());
    }

    /// Starts an analog to digital conversion that will be periodically
    /// triggered by the channel 2 of TIM2
    ///
    /// The conversions will be stored in the circular `buffer`
    pub fn start<B>(
        &self,
        buffer: &Static<CircBuffer<B, Dma2Stream0>>,
        dma2: &DMA2,
        pwm: Pwm<TIM2>,
    ) -> Result<(), dma::Error>
    where
        B: Unsize<[u16]>,
    {
        let adc1 = self.0;

        if dma2.s0cr.read().en().bit_is_set() {
            return Err(dma::Error::InUse);
        }

        pwm.disable(Channel::_2);
        pwm.set_duty(Channel::_2, 1);

        let buffer: &[u16] = &buffer.lock()[0];

        dma2.s0ndtr
            .write(|w| unsafe { w.ndt().bits(u16(buffer.len() * 2).unwrap()) });

        dma2.s0par
            .write(|w| unsafe { w.bits(&adc1.dr as *const _ as u32) });

        dma2.s0m0ar
            .write(|w| unsafe { w.bits(buffer.as_ptr() as u32) });

        dma2.s0cr.modify(|_, w| w.en().set_bit());
        pwm.enable(Channel::_2);

        Ok(())
    }
}
