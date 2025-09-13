//! Build compact JSON payloads without heap allocation.
use heapless::String;
use core::fmt::Write;
use crate::sensor::Env;

/// Create the JSON payload for the HTTP POST (field names match existing API).
///
/// The constant `N` is the max capacity of the returned string.
pub fn build_payload<const N: usize>(
    name: &str,
    rpi_temp_c: f32,
    env: Option<Env>,
    ip: &str,
    uptime_secs: u64,
) -> String<N> {
    let mut s: String<N> = String::new();

    let _ = write!(s, "{{\"name\":\"{}\",\"rpi_temp\":{:.1},", name, rpi_temp_c);

    match env {
        Some(e) => {
            let _ = write!(
                s,
                "\"temp\":{:.2},\"pressure\":{:.2},\"humidity\":{:.1},",
                e.temperature_c, e.pressure_hpa, e.humidity_pct
            );
        }
        None => {
            let _ = write!(s, "\"temp\":NaN,\"pressure\":NaN,\"humidity\":NaN,");
        }
    }

    let _ = write!(s, "\"ip_address\":\"{}\",\"uptime\":{}}}", ip, uptime_secs);
    s
}
