//! BME280 init helper + Env struct (no direct read function to avoid trait method mismatches).
// use bme280_multibus::{Bme280, Settings, Bme280Bus};
use defmt::Format;

/// One environmental reading.
#[derive(Clone, Copy, Debug, Format)]
pub struct Env {
    /// Temperature in °C.
    pub temperature_c: f32,
    /// Pressure in hPa.
    pub pressure_hpa: f32,
    /// Relative humidity in %.
    pub humidity_pct: f32,
}

// Apply BME280 settings and return the configured sensor.
// pub fn init_bme280<B>(
//     mut bme: Bme280<B>,
//     settings: &Settings,
// ) -> Result<Bme280<B>, bme280_multibus::Error<<B as Bme280Bus>::Error>>
// where
//     B: Bme280Bus,
// {
//     // Map bus error to crate error variant
//     bme.settings(settings).map_err(bme280_multibus::Error::Bus)?;
//     Ok(bme)
// }
