//! Peripheral bit banding

// TODO remove
#![allow(dead_code)]

use core::ptr;

pub unsafe fn write<T>(addr: *const T, bit: usize, set: bool) {
    let addr = addr as usize;
    assert!(addr >= 0x4000_0000 && addr < 0x4010_0000);
    let bb_addr = (0x4200_0000 + (addr as usize - 0x4000_0000) * 32) + 4 * bit;
    ptr::write_volatile(bb_addr as *mut u32, if set { 1 } else { 0 })
}
