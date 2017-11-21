//! Units of time

/// `b/s`, bits per second
#[derive(Clone, Copy, Debug)]
pub struct Bps(pub u32);

/// `Hz`
#[derive(Clone, Copy, Debug)]
pub struct Hertz(pub u32);

/// `us`
#[derive(Clone, Copy, Debug)]
pub struct Microseconds(pub u32);

/// `ms`
#[derive(Clone, Copy, Debug)]
pub struct Milliseconds(pub u32);

/// `s`
#[derive(Clone, Copy, Debug)]
pub struct Seconds(pub u32);

/// `u32` extension trait
pub trait U32Ext {
    /// Wrap in `Bps`
    fn bps(self) -> Bps;

    /// Wrap in `Hz`
    fn hz(self) -> Hertz;

    /// Wrap in `Milliseconds`
    fn ms(self) -> Milliseconds;

    /// Wrap in `Seconds`
    fn s(self) -> Seconds;

    /// Wrap in `Microseconds`
    fn us(self) -> Microseconds;
}

impl U32Ext for u32 {
    fn bps(self) -> Bps {
        Bps(self)
    }

    fn hz(self) -> Hertz {
        Hertz(self)
    }

    fn ms(self) -> Milliseconds {
        Milliseconds(self)
    }

    fn s(self) -> Seconds {
        Seconds(self)
    }

    fn us(self) -> Microseconds {
        Microseconds(self)
    }
}
