//! Wi‑Fi join helpers (CYW43).
use cyw43::Control;
use embassy_time::Timer;
use defmt::warn;

/// Join Wi‑Fi with infinite retry and a short delay between attempts.
///
/// Logs failures via `defmt`. Returns once associated.
pub async fn join_wifi(control: &mut Control<'_>, ssid: &str, pass: &str) {
    loop {
        match control.join(ssid, cyw43::JoinOptions::new(pass.as_bytes())).await {
            Ok(_) => break,
            Err(e) => {
                warn!("Wi‑Fi join failed: {}", e.status);
                Timer::after_millis(1000).await;
            }
        }
    }
}
