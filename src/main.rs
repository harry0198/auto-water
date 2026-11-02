#![no_std]
#![no_main]

use cyw43::aligned_bytes;
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_rp::adc::{Adc, Channel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler};
use embassy_rp::gpio::{Level, Output, Pull};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_rp::{bind_interrupts, dma};
use embassy_time::Timer;
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>;
    ADC_IRQ_FIFO => AdcInterruptHandler;
});

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

    // Sensor VCC on GP15, sensor AOUT on GP26 (ADC0)
    info!("here");
    let mut moisture_pwr = Output::new(p.PIN_16, Level::Low);
    let mut adc = Adc::new(p.ADC, Irqs, AdcConfig::default());
    let mut moisture_input = Channel::new_pin(p.PIN_26, Pull::None);
    let mut pump_pwt = Output::new(p.PIN_15, Level::Low);

    // pump_on.set_high();

    loop {
        info!("sensor on, reading...");
        control.gpio_set(0, true).await;
        moisture_pwr.set_high();
        Timer::after_millis(100).await; // let the sensor output settle

        // Average 10 samples to smooth out noise
        let mut total: u32 = 0;
        let mut count: u32 = 0;
        for _ in 0..10 {
            if let Ok(v) = adc.read(&mut moisture_input).await {
                total += v as u32;
                count += 1;
            }
        }

        moisture_pwr.set_low();
        control.gpio_set(0, false).await;

        if count > 0 {
            let raw = (total / count) as u16; // 12-bit: 0..=4095, lower = wetter
            info!("Moisture raw: {}", raw);
            if (raw < 1000) {
                println!("setting high");
                pump_pwt.set_high();
            } else {
                println!("setting low");
                pump_pwt.set_low();
            }
        } else {
            warn!("ADC read failed");
        }

        Timer::after_millis(3000).await;
    }
}