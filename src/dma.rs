use core::marker::PhantomData;

use stm32f30x::DMA1;

/// An on-going DMA transfer
// This is bit like a `Future` minus the panicking `poll` method
pub struct Transfer<C, P>
where
    C: Channel,
{
    channel: PhantomData<C>,
    payload: P,
}

impl<C, P> Transfer<C, P>
where
    C: Channel,
{
    pub(crate) unsafe fn new(_channel: C, payload: P) -> Self {
        Transfer {
            channel: PhantomData,
            payload: payload,
        }
    }
}

impl<P> Transfer<Dma1Channel4, P> {
    pub fn is_done(&self) -> Result<bool, Error> {
        let dma = unsafe { &*DMA1::ptr() };
        let isr = dma.isr.read();

        if isr.teif4().bit_is_set() {
            return Err(Error::Transfer);
        } else {
            return Ok(isr.tcif4().bit_is_set());
        }
    }

    // XXX does this need some sort of barrier?
    pub fn wait(self) -> Result<(Dma1Channel4, P), Error> {
        while !self.is_done()? {}

        let dma = unsafe { &*DMA1::ptr() };
        dma.ifcr.write(|w| w.ctcif4().set_bit());
        dma.ccr4.modify(|_, w| w.en().clear_bit());
        Ok((Dma1Channel4 { _0: () }, self.payload))
    }
}

impl<P> Transfer<Dma1Channel5, P> {
    pub fn is_done(&self) -> Result<bool, Error> {
        let dma = unsafe { &*DMA1::ptr() };
        let isr = dma.isr.read();

        if isr.teif5().bit_is_set() {
            return Err(Error::Transfer);
        } else {
            return Ok(isr.tcif5().bit_is_set());
        }
    }


    pub fn wait(self) -> Result<(Dma1Channel5, P), Error> {
        while !self.is_done()? {}

        let dma = unsafe { &*DMA1::ptr() };
        dma.ifcr.write(|w| w.ctcif5().set_bit());
        dma.ccr5.modify(|_, w| w.en().clear_bit());
        Ok((Dma1Channel5 { _0: () }, self.payload))
    }
}

#[derive(Debug)]
pub enum Error {
    Transfer,
}

pub trait DmaExt {
    type Channels;

    fn split(self) -> Self::Channels;
}

pub struct Dma1Channels(
    pub (),
    pub (),
    pub (),
    pub (),
    pub Dma1Channel4,
    pub Dma1Channel5,
);

impl DmaExt for DMA1 {
    type Channels = Dma1Channels;

    fn split(self) -> Dma1Channels {
        Dma1Channels(
            (),
            (),
            (),
            (),
            Dma1Channel4 { _0: () },
            Dma1Channel5 { _0: () },
        )
    }
}

pub trait Channel: 'static {}

pub struct Dma1Channel4 {
    _0: (),
}

pub struct Dma1Channel5 {
    _0: (),
}

impl Channel for Dma1Channel4 {}

impl Channel for Dma1Channel5 {}
