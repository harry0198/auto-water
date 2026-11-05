#![no_std]
#![no_main]

use cyw43::aligned_bytes;
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::adc::{Adc, Async, Channel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler};
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::{Timer};
use panic_probe as _;
use static_cell::StaticCell;


const CALIBRATION_MOISTURE_DRY: u32 = 2971;
const CALIBRATION_MOISTURE_WET: u32 = 800;
const MIN_SAMPLES: u8 = 6; // Amount of samples before reading is deemed inaccurate.


bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
    ADC_IRQ_FIFO => AdcInterruptHandler;
});

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MoistureError {
    /// Every ADC sample failed, so there was nothing to average.
    NoValidSamples,
    /// Some samples failed, but fewer than we require.
    TooFewSamples { got: u8, wanted: u8 },
}

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0>>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::main(executor = "embassy_rp::executor::Executor", entry = "cortex_m_rt::entry")]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // firmware blobs — copy from embassy-rs/embassy's top-level `cyw43-firmware/` dir
    let fw = aligned_bytes!("../cyw43-firmware/43439A0.bin");
    let clm = aligned_bytes!("../cyw43-firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../cyw43-firmware/nvram_rp2040.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        RM2_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        dma::Channel::new(p.DMA_CH0, Irqs),
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;
    spawner.spawn(unwrap!(cyw43_task(runner)));

    control.init(clm).await;
    control.set_power_management(cyw43::PowerManagementMode::PowerSave).await;

    let mut moisture_pwr = Output::new(p.PIN_16, Level::Low);
    let mut adc = Adc::new(p.ADC, Irqs, AdcConfig::default());
    let mut moisture_input = Channel::new_pin(p.PIN_26, Pull::None);
    let mut pump_pwt = Output::new(p.PIN_15, Level::Low);

    loop {
        info!("sensor on, reading...");
        control.gpio_set(0, true).await;
        Timer::after_millis(100).await; // let the sensor output settle

        let moisture_raw = get_moisture_raw(&mut moisture_pwr, &mut adc, &mut moisture_input).await;

        control.gpio_set(0, false).await;

        let moisture = match moisture_raw {
            Ok(v) => normalise_moisture(v),
            Err(MoistureError::NoValidSamples) => {
                warn!("Failed to sample any moisture readings");
                continue;
            },
            Err(MoistureError::TooFewSamples { got, wanted }) => {
                warn!("Failed to sample moisture readings. Got: {}, wanted: {}", got, wanted);
                continue;
            }
        };

        println!("moisture {}", &moisture);
       
        Timer::after_millis(3000).await;
    }
}

fn normalise_moisture(raw: u16) -> u8 {
    let dry = u32::from(CALIBRATION_MOISTURE_DRY);
    let wet = u32::from(CALIBRATION_MOISTURE_WET);

    let raw = u32::from(raw).clamp(wet, dry);
    (100 * (dry - raw) / (dry - wet)) as u8
}

async fn get_moisture_raw(power_pin: &mut Output<'_>, adc: &mut Adc<'_, Async>, pin: &mut Channel<'_>) -> Result<u16, MoistureError> {
    power_pin.set_high();

    Timer::after_millis(100).await;

    // Average 10 samples to smooth out noise
    let mut total: u32 = 0;
    let mut count: u8 = 0;
    for _ in 0..10 {
        if let Ok(v) = adc.read(pin).await {
            total += v as u32;
            count += 1;
        }
    }

    power_pin.set_low();

    match count {
        0 => Err(MoistureError::NoValidSamples),
        n if n < MIN_SAMPLES => Err(MoistureError::TooFewSamples { got: n, wanted: MIN_SAMPLES }),
        n => Ok((total / n as u32) as u16),
    }
}

