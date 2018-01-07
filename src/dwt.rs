//! Methods for trying a closure for a set number of clock cycles.

use stm32f40x::DWT;

/// Try a mutable closure until it returns Ok or times out.
pub fn try_mut_until<R, F>(dwt: &DWT, i: u32, mut f: F) -> Result<R, &'static str>
where
    F: FnMut() -> Option<R>,
{
    let cyccnt = dwt.cyccnt.read().wrapping_add(i);
    loop {
        match f() {
            Some(r) => break Ok(r),
            _ => (),
        }
        if (cyccnt.wrapping_sub(dwt.cyccnt.read()) as i32) < 0 {
            break Err(&"operation timed out");
        }
    }
}

/// Try an immutable closure until it returns Ok or times out.
pub fn try_until<R, F>(dwt: &DWT, i: u32, f: F) -> Result<R, &'static str>
where
    F: Fn() -> Option<R>,
{
    let cyccnt = dwt.cyccnt.read().wrapping_add(i);
    loop {
        match f() {
            Some(r) => break Ok(r),
            _ => (),
        }
        if (cyccnt.wrapping_sub(dwt.cyccnt.read()) as i32) < 0 {
            break Err(&"operation timed out");
        }
    }
}
