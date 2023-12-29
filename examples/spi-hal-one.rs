// This example is to test the SPI without any external devices.
// It puts "Hello world!" on the mosi-line and logs whatever is received on the miso-line to the info level.
// The idea is that you should connect miso and mosi, so you will also receive "Hello world!".
// It also transmits the data again using the old implementation, so those with a logic analyzer
// can appreciate the effects of prefilling the TX FIFO

#![no_main]
#![no_std]

use crate::hal::{
    prelude::*,
    pwr::PwrExt,
    rcc::Config,
    spi,
    stm32::Peripherals,
    time::RateExtU32,
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
    let mut rcc = rcc.freeze(
        Config::hsi(),
        pwr
    );

    // let gpioa = dp.GPIOA.split(&mut rcc);
    let gpioa = dp.GPIOA.split(&mut rcc);
    let sclk = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpioa.pa7.into_alternate();

    // 1/8 SPI/SysClk ratio seems to be the upper limit for continuous transmission
    // one byte at a time
    // 1/4 works for the first ~5 bytes (+4 prefilled), then we hit cpu limits
    let mut spi = dp
        .SPI1
        .spi((sclk, miso, mosi), spi::MODE_0, 2.MHz(), &mut rcc);
    let mut cs = gpioa.pa8.into_push_pull_output();
    cs.set_high().unwrap();

    // "Hello world!"
    const MESSAGE: &[u8] = "Hello world, but longer!".as_bytes();
    let received = &mut [0u8; MESSAGE.len()];

    cs.set_low().unwrap();
    SpiBus::transfer(&mut spi, received, MESSAGE).unwrap();
    spi.flush().unwrap();
    cs.set_high().unwrap();

    cs.set_low().unwrap();
    embedded_hal::blocking::spi::Write::write(&mut spi, MESSAGE).unwrap();
    cs.set_high().unwrap();
    info!("Received {:?}", core::str::from_utf8(received).ok());
    assert_eq!(MESSAGE, received);

    loop {
        cortex_m::asm::nop();
    }
}
