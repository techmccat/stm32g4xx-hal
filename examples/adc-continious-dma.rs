#![no_std]
#![no_main]

mod utils;
use utils::logger::info;

use crate::hal::{
    adc::{
        config,
        config::{Continuous, Dma as AdcDma, SampleTime, Sequence},
        AdcClaim, ClockSource, Temperature, Vref,
    },
    delay::DelayFromCountDownTimer,
    dma::{config::DmaConfig, stream::DMAExt, TransferExt},
    gpio::GpioExt,
    pwr::PwrExt,
    rcc::{Config, RccExt},
    signature::{VrefCal, VDDA_CALIB},
    stm32::Peripherals,
    time::ExtU32,
    timer::Timer,
};
use stm32g4xx_hal as hal;

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    utils::logger::init();

    info!("start");

    let dp = Peripherals::take().unwrap();
    // let cp = cortex_m::Peripherals::take().expect("cannot take core peripherals");

    info!("rcc");
    let rcc = dp.RCC.constrain();
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = rcc.freeze(Config::hsi(), pwr);

    let streams = dp.DMA1.split(&rcc);
    let config = DmaConfig::default()
        .transfer_complete_interrupt(false)
        .circular_buffer(true)
        .memory_increment(true);

    info!("Setup Gpio");
    let gpioa = dp.GPIOA.split(&mut rcc);
    let pa0 = gpioa.pa0.into_analog();

    info!("Setup Adc1");
    // let mut delay = cp.SYST.delay(&rcc.clocks);
    let mut delay = DelayFromCountDownTimer::new(
        Timer::new(dp.TIM6, &rcc.clocks).start_count_down(100u32.millis()),
    );
    let mut adc = dp
        .ADC1
        .claim(ClockSource::SystemClock, &rcc, &mut delay, true);

    adc.enable_temperature(&dp.ADC12_COMMON);
    adc.enable_vref(&dp.ADC12_COMMON);
    adc.set_continuous(Continuous::Continuous);
    adc.reset_sequence();
    adc.configure_channel(&pa0, Sequence::One, SampleTime::Cycles_640_5);
    adc.configure_channel(&Temperature, Sequence::Two, SampleTime::Cycles_640_5);
    adc.configure_channel(&Vref, Sequence::Three, SampleTime::Cycles_640_5);

    info!("Setup DMA");
    let first_buffer = cortex_m::singleton!(: [u16; 15] = [0; 15]).unwrap();
    let mut transfer = streams.0.into_circ_peripheral_to_memory_transfer(
        adc.enable_dma(AdcDma::Continuous),
        &mut first_buffer[..],
        config,
    );

    transfer.start(|adc| adc.start_conversion());

    loop {
        let mut b = [0_u16; 6];
        let r = transfer.read_exact(&mut b);
        assert!(
            !transfer.get_overrun_flag(),
            "DMA did not have time to read the ADC value before ADC was done with a new conversion"
        );

        info!("read: {}", r);
        assert!(r == b.len());

        let vdda = VDDA_CALIB * VrefCal::get().read() as u32 / ((b[2] + b[5]) / 2) as u32;

        info!("vdda: {}mV", vdda);

        let millivolts =
            Vref::sample_to_millivolts_ext((b[0] + b[3]) / 2, vdda, config::Resolution::Twelve);
        info!("pa0: {}mV", millivolts);
        let vref =
            Vref::sample_to_millivolts_ext((b[2] + b[5]) / 2, vdda, config::Resolution::Twelve);
        info!("vref: {}mV", vref);
        let temp = Temperature::temperature_to_degrees_centigrade(
            (b[1] + b[4]) / 2,
            vdda as f32 / 1000.,
            config::Resolution::Twelve,
        );
        info!("temp: {}°C", temp);
    }
}
