#![deny(warnings)]
#![deny(unsafe_code)]
#![no_main]
#![no_std]

use embedded_graphics::mono_font::MonoTextStyle;
use hal::i2c::Config;
use hal::prelude::*;
use hal::stm32;
use hal::time::RateExtU32;
use stm32g4xx_hal as hal;

use cortex_m_rt::entry;

use sh1106::{prelude::*, Builder};
use embedded_graphics::{
    mono_font::ascii::FONT_6X10,
    pixelcolor::BinaryColor,
    prelude::*,
    text::Text,
};

#[macro_use]
mod utils;

#[entry]
fn main() -> ! {
    utils::logger::init();

    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();
    let gpiob = dp.GPIOB.split(&mut rcc);

    let sda = gpiob.pb9.into_alternate_open_drain();
    let scl = gpiob.pb8.into_alternate_open_drain();

    let i2c = dp.I2C1.i2c(sda, scl, Config::new(100.kHz()), &mut rcc);
    // Alternatively, it is possible to specify the exact timing as follows (see the documentation
    // of with_timing() for an explanation of the constant):
    //let mut i2c = dp
    //  .I2C1
    //   .i2c(sda, scl, Config::with_timing(0x3042_0f13), &mut rcc);

    let mut disp: GraphicsMode<_> = Builder::new().connect_i2c(i2c).into();
    disp.init().unwrap();
    disp.flush().unwrap();

    let style = MonoTextStyle::new(&FONT_6X10, BinaryColor::On);
    Text::new("Hello Rust!", Point::new(16, 16), style)
        .draw(&mut disp)
        .unwrap();

    disp.flush().unwrap();

    loop {}
}
