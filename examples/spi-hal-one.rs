// This example is to test the SPI without any external devices.
// It puts "Hello world!" on the mosi-line and logs whatever is received on the miso-line to the info level.
// The idea is that you should connect miso and mosi, so you will also receive "Hello world!".

#![no_main]
#![no_std]

use crate::hal::{
    delay::DelayFromCountDownTimer,
    prelude::*,
    pwr::PwrExt,
    rcc::Config,
    spi,
    stm32::Peripherals,
    time::{ExtU32, RateExtU32},
    timer::Timer,
};

use cortex_m_rt::entry;
use embedded_hal_one::spi::SpiBus;
use stm32g4xx_hal as hal;

#[macro_use]
mod utils;
use utils::logger::info;

#[entry]
fn main() -> ! {
    utils::logger::init();
    info!("Logger init");

    let dp = Peripherals::take().unwrap();
    let rcc = dp.RCC.constrain();
    let pwr = dp.PWR.constrain().freeze();
    let mut rcc = rcc.freeze(Config::hsi(), pwr);
    let timer2 = Timer::new(dp.TIM2, &rcc.clocks);
    let mut delay_tim2 = DelayFromCountDownTimer::new(timer2.start_count_down(100.millis()));

    // let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let sclk = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();

    let mut spi = dp
        .SPI1
        .spi((sclk, miso, mosi), spi::MODE_0, 400.kHz(), &mut rcc);
    let mut cs = gpioa.pa8.into_push_pull_output();
    cs.set_high().unwrap();

    // "Hello world!"
    const MESSAGE: &[u8] = "Hello world!".as_bytes();
    let received = &mut [0u8; MESSAGE.len()];

    cs.set_low().unwrap();
    SpiBus::transfer(&mut spi, received, MESSAGE).unwrap();
    spi.flush().unwrap();
    cs.set_high().unwrap();

    info!("Received {:?}", core::str::from_utf8(received).ok());
    delay_tim2.delay_ms(10_u16);

    cs.set_low().unwrap();
    embedded_hal::blocking::spi::Write::write(&mut spi, received).unwrap();
    cs.set_high().unwrap();

    // info!("{:?}", core::str::from_utf8(received).ok());
    loop {
        cortex_m::asm::nop();
    }
}
