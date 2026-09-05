#![no_main]
#![no_std]

mod utils;

#[rtic::app(
    device = stm32g4xx_hal::pac,
    dispatchers = [I2C2_EV, I2C2_ER]
)]
mod app {
    use crate::utils::logger;

    use embedded_hal::spi::MODE_1;
    use embedded_hal_async::spi::SpiBus;

    use fugit::RateExtU32;
    use stm32g4xx_hal::{
        self as hal, gpio::{AF5, PB3, PB4, PB5}, pac::SPI1, prelude::*, rcc, spi::{Spi, SpiAsyncBasic, SpiIrqBasic}
    };


    #[shared]
    struct Shared { }

    #[local]
    struct Local {
        spi_irq: SpiIrqBasic<SPI1>,
        spi_async: SpiAsyncBasic<
            SPI1, (PB3<AF5>, PB4<AF5>, PB5<AF5>)>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        logger::init();
        let pwr = cx.device.PWR.constrain().freeze();
        // 16MHz sysclk
        let mut rcc = cx.device.RCC
            .constrain()
            .freeze(rcc::Config::hsi(), pwr);

        let gpiob = cx.device.GPIOB.split(&mut rcc);

        let sck = gpiob.pb3.into_alternate();
        let miso = gpiob.pb4.into_alternate();
        let mosi = gpiob.pb5.into_alternate();
        let pins = (sck, miso, mosi);

        let spi = cx.device.SPI1.spi(pins, MODE_1, 2u32.MHz(), &mut rcc);
        let (spi_async, spi_irq) = spi.into_async_basic();

        transmit::spawn().ok();

        (
            Shared { },
            Local {
                spi_async,
                spi_irq
            }
        )
    }

    #[task(priority = 1, local = [spi_async])]
    async fn transmit(cx: transmit::Context) {
        let tx_buf: [u8; 128] = core::array::from_fn(|i| 
            if i.is_multiple_of(2) { 0b10101100 } else { 0b11110000 }
        );

        loop {
            logger::info!("tx start");
            cx.local.spi_async.write(&tx_buf).await.unwrap();
            cx.local.spi_async.flush().await.unwrap();
        }
    }

    #[task(priority = 2, binds = SPI1, local = [spi_irq])]
    fn spi1(cx: spi1::Context) {
        cx.local.spi_irq.on_irq();
    }
}
