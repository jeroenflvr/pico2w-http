//! LED blink helpers for small, readable patterns.
use embassy_time::{Duration, Timer};

/// Blink an LED `count` times with `on_ms`/`off_ms` timing.
///
/// Returns when the pattern is complete.
pub async fn blink_led(
    led: &mut embassy_rp::gpio::Output<'static>,
    count: usize,
    on_ms: u64,
    off_ms: u64,
) {
    for _ in 0..count {
        led.set_high();
        Timer::after(Duration::from_millis(on_ms)).await;
        led.set_low();
        Timer::after(Duration::from_millis(off_ms)).await;
    }
}


// Turn LED on for `ms` milliseconds (useful as a simple wait with a visual cue).
// pub async fn solid_for(
//     led: &mut embassy_rp::gpio::Output<'static>,
//     ms: u64,
// ) {
//     led.set_high();
//     Timer::after(Duration::from_millis(ms)).await;
//     led.set_low();
// }
