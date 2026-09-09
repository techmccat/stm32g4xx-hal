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
        gpio::{AF5, PB3, PB4, PB5, PB13, PB14, PB15},
        pac::{SPI1, SPI2},
        prelude::*,
        rcc,
        spi::{SpiAsyncBasic, SpiAsyncFast, SpiIrqBasic, SpiIrqFast}
    };

    type Spi1Pins = (PB3<AF5>, PB4<AF5>, PB5<AF5>);
    type Spi2Pins = (PB13<AF5>, PB14<AF5>, PB15<AF5>);

    #[shared]
    struct Shared { }

    #[local]
    struct Local {
        spi1_irq: SpiIrqBasic<SPI1>,
        spi1_async: SpiAsyncBasic<SPI1, Spi1Pins>,
        spi2_irq: SpiIrqFast<SPI2>,
        spi2_async: SpiAsyncFast<SPI2, Spi2Pins>,
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

        let pins1 = {
            let sck = gpiob.pb3.into_alternate();
            let miso = gpiob.pb4.into_alternate();
            let mosi = gpiob.pb5.into_alternate();
            (sck, miso, mosi)
        };
        let pins2 = {
            let sck = gpiob.pb13.into_alternate();
            let miso = gpiob.pb14.into_alternate();
            let mosi = gpiob.pb15.into_alternate();
            (sck, miso, mosi)
        };

        let spi1 = cx.device.SPI1.spi(pins1, MODE_1, 2u32.MHz(), &mut rcc);
        let (spi1_async, spi1_irq) = spi1.into_async_basic();
        let spi2 = cx.device.SPI2.spi(pins2, MODE_1, 4u32.MHz(), &mut rcc);
        let (spi2_async, spi2_irq) = spi2.into_async_fast().unwrap();

        transmit::spawn().ok();

        (
            Shared { },
            Local {
                spi1_async,
                spi1_irq,
                spi2_async,
                spi2_irq,
            }
        )
    }

    #[task(priority = 1, local = [spi1_async, spi2_async])]
    async fn transmit(cx: transmit::Context) {
        let tx_buf: [u8; 127] = core::array::from_fn(|i| 
            if i.is_multiple_of(2) { 0b10101100 } else { 0b11110000 }
        );

        loop {
            logger::info!("tx start");
            cx.local.spi1_async.write(&tx_buf).await.unwrap();
            cx.local.spi1_async.flush().await.unwrap();
            cx.local.spi2_async.write(&tx_buf).await.unwrap();
            cx.local.spi2_async.flush().await.unwrap();
        }
    }

    #[task(priority = 2, binds = SPI1, local = [spi1_irq])]
    fn spi1(cx: spi1::Context) {
        cx.local.spi1_irq.on_irq();
    }
    #[task(priority = 2, binds = SPI2, local = [spi2_irq])]
    fn spi2(cx: spi2::Context) {
        // safety: higher priority than `transmit`
        unsafe { cx.local.spi2_irq.on_irq() };
    }
}
