//! Try a closure until a certain number of clock cycles has passed
#![deny(unsafe_code)]
#![feature(proc_macro)]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rtfm as rtfm;
extern crate f4;
extern crate heapless;

use cortex_m::peripheral::SystClkSource;
use f4::led::{self, LED};
use f4::clock;
use f4::dwt;
use rtfm::{app, Threshold};

// CONFIGURATION
const FREQUENCY: u32 = 10; // Hz

// TASKS & RESOURCES
app! {
    device: f4::stm32f40x,

    resources: {
        static INTEGRATOR: u32 = 0;
        static CLK: u32 = 0;
    },
    idle: {
        resources: [DWT, CLK, INTEGRATOR],
    },
    tasks: {
        SYS_TICK: {
            path: tick,
            resources: [INTEGRATOR],
        },
    },
}

// INITIALIZATION PHASE
fn init(p: init::Peripherals, r: init::Resources) {
    // Try clocking to see that it works. Store the frequency as a resource.
    **r.CLK = clock::set_84_mhz(&p.RCC, &p.FLASH);
    // Initialize the user LED
    led::init(p.GPIOA, p.RCC);
    // Start the systick timer
    p.SYST.set_clock_source(SystClkSource::Core);
    p.SYST.set_reload(**r.CLK / FREQUENCY);
    p.SYST.enable_interrupt();
    p.SYST.enable_counter();
    // We must enable the cyccnt for try_until to work
    p.DWT.enable_cycle_counter();
}

// IDLE LOOP
fn idle(t: &mut Threshold, r: ::idle::Resources) -> ! {
    use rtfm::Resource;

    // Try the closure for 50 ms (20 Hz)
    let try_cycles = *r.CLK / 20;

    // Infinite loop
    loop {
        // Try until the closure for a set number of clock cycles.
        // We need to capture the mutable Threshold t, so use the mutable try.
        match dwt::try_mut_until(&r.DWT, try_cycles, &mut || {
            // Claim the integrator long enough to clone it
            let integrator = r.INTEGRATOR.claim(t, |v, _| **v.clone());
            // The integrator becomes even every 200 ms since the
            // systick interrupt runs every 100 ms (10 Hz).
            if integrator % 2 == 0 {
                Some(integrator)
            } else {
                None
            }
        }) {
            // The tried closure timed out.
            Err(_) => LED.off(),
            // The tried closure returned something
            Ok(i) => {
                assert_eq!(i % 2, 0);
                LED.on()
            }
        }
        // The try is matched to Ok immediately if the integrator is even
        // but tries it for 50 ms if it is not. This should result in the LED
        // blinking at 5 Hz (interrupted at 10 Hz) with a duty cycle of 75%.
    }
}

// Interrupt for systick which mutates a shared resource
fn tick(_t: &mut Threshold, r: SYS_TICK::Resources) {
    **r.INTEGRATOR = (**r.INTEGRATOR).wrapping_add(1);
}