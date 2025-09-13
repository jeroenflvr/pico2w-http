#![no_std]
#![no_main]
#![allow(async_fn_in_trait)]

// --- pico2w modules ---
mod blink;
mod wifi;
mod sensor;
mod http;
mod payload;
mod utils;

use crate::blink::blink_led;
use crate::wifi::join_wifi;
use crate::utils::convert_to_celsius;


use core::str::from_utf8;
use core::fmt::Write as _;


use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::{Config, IpAddress, IpEndpoint, Ipv4Address, StackResources};
use embassy_net::tcp::TcpSocket;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_time::{Duration, Timer};
use heapless::String;
use static_cell::StaticCell;

// use bme280_multibus::{Bme280, Sample};

// SPI (blocking) for Embassy RP
use embassy_rp::spi::{self, Spi, Config as SpiConfig};
// use bme280_multibus::{Bme280, Sample, Config as bme280_CONFIG, Settings, CtrlMeas, Standby, Filter, Oversampling, Mode, CHIP_ID};
use bme280_multibus::{Bme280, Sample};

use embedded_hal_bus::spi::ExclusiveDevice;

use embassy_time::Delay;


// Bring in RTT logging and a panic handler unconditionally.
use defmt_rtt as _;
use panic_probe as _;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});


#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(
    mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>
) -> ! {
    runner.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("boot");

    const WIFI_SSID: &str = env!("WIFI_SSID");
    const WIFI_PASS: &str = env!("WIFI_PASS");

    // These are compile-time strings, parse at runtime
    let endpoint_host = env!("ENDPOINT_HOST");
    let endpoint_port: u16 = env!("ENDPOINT_PORT").parse().expect("ENDPOINT_PORT must be a valid u16");
    let device_name = env!("DEVICE_NAME");
    let sensors_route = env!("SENSORS_ROUTE");
    let p = embassy_rp::init(Default::default());

    // Firmware blobs
    let fw  = include_bytes!("../firmware/43439A0.bin");
    let clm = include_bytes!("../firmware/43439A0_clm.bin");

    // PIO SPI wiring for Pico 2W / RM2
    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs  = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);

    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        RM2_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24, // MOSI
        p.PIN_29, // SCK
        p.DMA_CH0,
    );
    let mut led15 = Output::new(p.PIN_15, Level::Low);

    // BME280 on SPI config
    // Wiring: SCK=GPIO18, MOSI=GPIO19, MISO=GPIO16, CS=GPIO17
    let sck  = p.PIN_18;
    let mosi = p.PIN_19;
    let miso = p.PIN_16;

    // Chip-select (active-low), start high (not selected)
    let cs_bme = Output::new(p.PIN_17, Level::High);

    // SPI0 config: BME280 uses MODE 0 -> IdleLow + CaptureOnFirstTransition
    let mut spi_cfg = SpiConfig::default();
    spi_cfg.frequency = 1_000_000; // 1 MHz is safe
    spi_cfg.polarity = spi::Polarity::IdleLow;
    spi_cfg.phase    = spi::Phase::CaptureOnFirstTransition;

    // Blocking SPI is simplest
    let spi_bme = Spi::new_blocking(p.SPI0, sck, mosi, miso, spi_cfg);

    // Wrap SPI bus + CS into an eh1 SpiDevice
    let spi_dev = ExclusiveDevice::new(spi_bme, cs_bme, Delay);

    // Create the BME280 driver (eh1 SpiDevice)
    let mut bme = Bme280::from_spi1(spi_dev)
        .expect("BME280 init failed");

    embassy_time::Timer::after_millis(10).await; // tiny power-up settle, safe to keep

    blink_led(&mut led15, 3, 200, 200).await;

    const SETTINGS: bme280_multibus::Settings = bme280_multibus::Settings {
        config: bme280_multibus::Config::RESET
            .set_standby_time(bme280_multibus::Standby::Millis125)
            .set_filter(bme280_multibus::Filter::X8),
        ctrl_meas: bme280_multibus::CtrlMeas::RESET
            .set_osrs_t(bme280_multibus::Oversampling::X8)
            .set_osrs_p(bme280_multibus::Oversampling::X8)
            .set_mode(bme280_multibus::Mode::Normal),
        ctrl_hum: bme280_multibus::Oversampling::X8,
    };

    blink_led(&mut led15, 3, 200, 200).await;

    bme.settings(&SETTINGS).expect("BME280: settings failed");

    blink_led(&mut led15, 5, 800, 200).await;

    // CYW43 bring-up
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (net_dev, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    // NOTE: unwrap the token BEFORE passing to spawn; spawn returns ()
    let _ = spawner.spawn(cyw43_task(runner));

    control.init(clm).await;
    control.set_power_management(cyw43::PowerManagementMode::PowerSave).await;

    // Network stack (DHCP)
    static RES: StaticCell<StackResources<5>> = StaticCell::new();
    let config = Config::dhcpv4(Default::default());
    let seed: u64 = 0x1234_5678_9ABC_DEF0;
    let (stack, net_runner) = embassy_net::new(net_dev, config, RES.init(StackResources::new()), seed);
    let _ = spawner.spawn(net_task(net_runner));

    // Join Wi-Fi
    // ADC setup for temperature
    use embassy_rp::adc::{Adc, Channel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler};
    bind_interrupts!(struct AdcIrqs {
        ADC_IRQ_FIFO => AdcInterruptHandler;
    });
    let mut adc = Adc::new(p.ADC, AdcIrqs, AdcConfig::default());
    let mut ts = Channel::new_temp_sensor(p.ADC_TEMP_SENSOR);
    // let mut led15 = Output::new(p.PIN_15, Level::Low);
    let start = embassy_time::Instant::now();
    join_wifi(&mut control, WIFI_SSID, WIFI_PASS).await;


    info!("Wi-Fi joined; waiting link/DHCP");
    stack.wait_link_up().await;
    stack.wait_config_up().await;
    info!("Network up");


    // ---- HTTP POST (raw TCP) ----
    // Parse endpoint_host as IPv4 address at runtime
    let host_bytes: heapless::Vec<u8, 4> = endpoint_host
        .split('.')
        .map(|s| s.parse::<u8>().expect("ENDPOINT_HOST must be IPv4"))
        .collect();
    ::core::assert!(host_bytes.len() == 4, "ENDPOINT_HOST must have 4 octets");
    let remote = IpEndpoint::new(
        IpAddress::Ipv4(Ipv4Address::new(host_bytes[0], host_bytes[1], host_bytes[2], host_bytes[3])),
        endpoint_port
    );

    let mut rx = [0u8; 1024];
    let mut tx = [0u8; 1024];
    let mut buf = [0u8; 256];

    loop {
        // Blink once before sending (250ms on, 250ms off)
        control.gpio_set(0, true).await;
        Timer::after(Duration::from_millis(250)).await;
        control.gpio_set(0, false).await;
        Timer::after(Duration::from_millis(250)).await;

        // Read BME280 sample (°C, Pa, %RH)
        let sample = bme.sample().unwrap_or(Sample {
            temperature: core::f32::NAN,
            pressure:    core::f32::NAN,
            humidity:    core::f32::NAN,
        });

        let bme_temp_c    = sample.temperature as f32;
        let bme_press_hpa = (sample.pressure as f32) / 100.0; // Pa -> hPa
        let bme_hum       = sample.humidity as f32;


        // Measure temperature
        let raw_temp = adc.read(&mut ts).await.unwrap();
        let temp_c = convert_to_celsius(raw_temp);

        // Get IP address
        let ip = match stack.config_v4() {
            Some(cfg) => cfg.address.address(),
            None => Ipv4Address::new(0,0,0,0),
        };
        // Calculate uptime in seconds
        let uptime = (embassy_time::Instant::now() - start).as_secs();

        // Build JSON body via module helper
        let env = crate::sensor::Env { temperature_c: bme_temp_c, pressure_hpa: bme_press_hpa, humidity_pct: bme_hum };
        let mut ip_s: heapless::String<24> = heapless::String::new();
        let _ = core::write!(&mut ip_s, "{}", ip);
        let body = crate::payload::build_payload::<256>(
            device_name,
            temp_c,
            Some(env),
            ip_s.as_str(),
            uptime
        );

        let mut req: String<256> = String::new();
        core::write!(
            &mut req,
            "POST {} HTTP/1.1\r\n\
             Host: {}:{}\r\n\
             Content-Type: application/json\r\n\
             Content-Length: {}\r\n\
             Connection: close\r\n\
             \r\n",
            sensors_route,
            endpoint_host,
            endpoint_port,
            body.len()
        ).unwrap();
        req.push_str(&body).unwrap();

        // Open TCP and send
        let mut sock = TcpSocket::new(stack, &mut rx, &mut tx);
        let mut host_hdr: heapless::String<64> = heapless::String::new();
        let _ = core::write!(&mut host_hdr, "{}:{}", endpoint_host, endpoint_port);
        if let Err(e) = crate::http::post_json::<256>(&mut sock, remote, host_hdr.as_str(), sensors_route, &body).await {
            warn!("http post error: {}", e);
        }

        // Optional: read a bit of the response
        if let Ok(n) = sock.read(&mut buf).await {
            if n > 0 {
                if let Ok(txt) = from_utf8(&buf[..n]) {
                    info!("resp: {}", txt);
                }
            }
        }
        sock.close();

        // Blink 3 times after sending (100ms on, 100ms off)
        for _ in 0..3 {
            control.gpio_set(0, true).await;
            Timer::after(Duration::from_millis(100)).await;
            control.gpio_set(0, false).await;
            Timer::after(Duration::from_millis(100)).await;
        }

        // Wait for the remainder of 5 seconds (5s - 250ms*2 - 100ms*6 = 3.9s)
        led15.set_high();
        Timer::after(Duration::from_millis(3900)).await;
        led15.set_low();
    }
}