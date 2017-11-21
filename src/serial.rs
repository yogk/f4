use core::marker::Unsize;
use core::ptr;

use cast::u16;
use hal;
use nb;
use stm32f30x::{DMA1, USART1};

use apb1;
use dma::{Dma1Channel4, Dma1Channel5, Transfer};
use gpio::{Af7, PA10, PA9};

#[derive(Debug)]
pub enum Error {
    Framing,
    Noise,
    Overrun,
    Parity,
}

pub enum Event {
    Tc,
    Txe,
    Rxne,
}

pub struct Rx {
    _0: (),
}

impl Rx {
    pub fn read_exact<B>(
        self,
        channel: Dma1Channel5,
        buffer: &'static mut B,
    ) -> Transfer<Dma1Channel5, (Rx, &'static mut B)>
    where
        B: Unsize<[u8]>,
    {
        let dma = unsafe { &*DMA1::ptr() };

        dma.cpar5.write(|w| unsafe {
            w.bits(&(*USART1::ptr()).rdr as *const _ as u32)
        });

        {
            let slice: &mut [u8] = &mut *buffer;

            dma.cndtr5
                .write(|w| unsafe { w.ndt().bits(u16(slice.len()).unwrap()) });
            dma.cmar5
                .write(|w| unsafe { w.bits(slice.as_ptr() as u32) });
        }

        dma.ccr5.write(|w| unsafe {
            w.mem2mem()
                .clear_bit()
                .pl()
                .bits(0b01)
                .msize()
                .bits(0b00)
                .psize()
                .bits(0b00)
                .minc()
                .set_bit()
                .circ()
                .clear_bit()
                .pinc()
                .clear_bit()
                .dir()
                .clear_bit()
                .tcie()
                .set_bit()
                .en()
                .set_bit()
        });

        unsafe { Transfer::new(channel, (self, buffer)) }
    }
}

pub struct Tx {
    _0: (),
}

// TODO it should be possible to *read* the buffer while the transfer is on going
impl Tx {
    pub fn write_all<B>(
        self,
        channel: Dma1Channel4,
        buffer: &'static mut B,
    ) -> Transfer<Dma1Channel4, (Tx, &'static mut B)>
    where
        B: Unsize<[u8]>,
    {
        let dma = unsafe { &*DMA1::ptr() };

        {
            let slice: &mut [u8] = &mut *buffer;

            dma.cndtr4
                .write(|w| unsafe { w.ndt().bits(u16(slice.len()).unwrap()) });
            dma.cpar4.write(|w| unsafe {
                w.bits(&(*USART1::ptr()).tdr as *const _ as u32)
            });
            dma.cmar4
                .write(|w| unsafe { w.bits(slice.as_ptr() as u32) });
        }

        dma.ccr4.write(|w| unsafe {
            w.mem2mem()
                .clear_bit()
                .pl()
                .bits(0b01)
                .msize()
                .bits(0b00)
                .psize()
                .bits(0b00)
                .minc()
                .set_bit()
                .circ()
                .clear_bit()
                .pinc()
                .clear_bit()
                .dir()
                .set_bit()
                .tcie()
                .set_bit()
                .en()
                .set_bit()
        });

        unsafe { Transfer::new(channel, (self, buffer)) }
    }
}

pub struct Serial {
    usart: USART1,
}

impl Serial {
    pub fn new<B>(usart: USART1, (_tx, _rx): (Af7<PA9>, Af7<PA10>), baud_rate: B) -> Self
    where
        B: Into<apb1::Ticks>,
    {
        // 1 stop bit
        usart.cr2.write(|w| unsafe { w.stop().bits(0b00) });

        // baud rate
        let brr = baud_rate.into().0;
        assert!(brr >= 16, "impossible baud rate");
        usart.brr.write(|w| unsafe { w.bits(brr) });

        // disable hardware flow control
        // enable DMA TX & RX transfers
        usart.cr3.write(|w| {
            w.rtse()
                .clear_bit()
                .ctse()
                .clear_bit()
                .dmat()
                .set_bit()
                .dmar()
                .set_bit()
        });

        // enable TX, RX; disable parity checking
        usart.cr1.write(|w| {
            w.ue()
                .set_bit()
                .re()
                .set_bit()
                .te()
                .set_bit()
                .m()
                .clear_bit()
                .over8()
                .clear_bit()
                .pce()
                .clear_bit()
                .rxneie()
                .clear_bit()
        });

        Serial { usart }
    }

    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().set_bit()),
            Event::Tc => self.usart.cr1.modify(|_, w| w.tcie().set_bit()),
            Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().set_bit()),
        }
    }

    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Rxne => self.usart.cr1.modify(|_, w| w.rxneie().clear_bit()),
            Event::Tc => self.usart.cr1.modify(|_, w| w.tcie().clear_bit()),
            Event::Txe => self.usart.cr1.modify(|_, w| w.txeie().clear_bit()),
        }
    }

    pub fn split(self) -> (Tx, Rx) {
        (Tx { _0: () }, Rx { _0: () })
    }
}

impl hal::serial::Read<u8> for Serial {
    type Error = Error;

    fn read(&mut self) -> nb::Result<u8, Error> {
        let sr = self.usart.isr.read();

        if sr.pe().bit_is_set() {
            Err(nb::Error::Other(Error::Parity))
        } else if sr.ore().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.nf().bit_is_set() {
            Err(nb::Error::Other(Error::Noise))
        } else if sr.fe().bit_is_set() {
            Err(nb::Error::Other(Error::Framing))
        } else if sr.rxne().bit_is_set() {
            Ok(unsafe {
                ptr::read_volatile(&self.usart.rdr as *const _ as *const u8)
            })
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl hal::serial::Write<u8> for Serial {
    type Error = Error;

    fn write(&mut self, byte: u8) -> nb::Result<(), Error> {
        let sr = self.usart.isr.read();

        if sr.pe().bit_is_set() {
            Err(nb::Error::Other(Error::Parity))
        } else if sr.ore().bit_is_set() {
            Err(nb::Error::Other(Error::Overrun))
        } else if sr.nf().bit_is_set() {
            Err(nb::Error::Other(Error::Noise))
        } else if sr.fe().bit_is_set() {
            Err(nb::Error::Other(Error::Framing))
        } else if sr.txe().bit_is_set() {
            Ok(unsafe {
                ptr::write_volatile(&self.usart.tdr as *const _ as *mut u8, byte)
            })
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}
