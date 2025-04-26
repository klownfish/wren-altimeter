#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[allow(unused_imports)]
#[cfg(target_os = "none")]
use defmt::{debug, error, info, trace, warn};
#[cfg(target_os = "none")]
use dummy_pin::DummyPin;
#[cfg(target_os = "none")]
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
#[cfg(target_os = "none")]
use embassy_nrf::gpio::{Level, Output, OutputDrive};
#[cfg(target_os = "none")]
use embassy_nrf::peripherals::SPI2;
#[cfg(target_os = "none")]
use embassy_nrf::saadc;
#[cfg(target_os = "none")]
use embassy_nrf::{bind_interrupts, peripherals, spim, wdt};
#[cfg(target_os = "none")]
#[cfg(feature = "wren")]
use embassy_nrf::usb;
#[cfg(target_os = "none")]
use embassy_time::Delay;
#[cfg(target_os = "none")]
use embassy_time::with_timeout;
#[cfg(target_os = "none")]
use embassy_nrf::pac;
#[cfg(target_os = "none")]
use libm::powf;
#[cfg(target_os = "none")]
use {defmt_rtt as _, panic_probe as _};

#[cfg(all(target_os = "none", feature = "wren-bt"))]
use embassy_nrf::peripherals::RNG;
#[cfg(all(target_os = "none", feature = "wren-bt"))]
use embassy_nrf::rng;

#[cfg(all(target_os = "none", feature = "wren-bt"))]
use nrf_sdc::mpsl::MultiprotocolServiceLayer;
#[cfg(all(target_os = "none", feature = "wren-bt"))]
use nrf_sdc::{self as sdc, mpsl};

#[allow(unused_imports)]
#[cfg(not(target_os = "none"))]
use log::{debug, error, info, trace, warn};
#[cfg(not(target_os = "none"))]
use env_logger;

use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use flash_writer::FlashWriter;
use platform::{AccelerometerType, BarometerType, FlashMemoryType};
use static_cell::StaticCell;


mod flash_writer;
mod flight_sm;
mod flight_task;
mod kalman;
mod platform;

#[cfg(all(target_os = "none", feature = "wren"))]
mod usb_handler;

#[cfg(all(target_os = "none", feature = "wren-bt"))]
mod bluetooth;
#[cfg(all(target_os = "none", feature = "wren-bt"))]
mod nus;

#[allow(unused)]
struct WrenState {
    battery_voltage: f32,
    acceleration: f32,
    altitude: f32,
}

#[cfg(all(feature = "wren", feature = "wren-bt"))]
compile_error!("Features 'wren' and 'wren-bt' are mutually exclusive!");

#[cfg(target_os = "none")]
bind_interrupts!(struct Irqs {
    #[cfg(feature = "wren")]
    USBD => usb::InterruptHandler<peripherals::USBD>;
    #[cfg(feature = "wren")]
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;

    SPI2 => spim::InterruptHandler<SPI2>;
    SAADC => saadc::InterruptHandler;

    #[cfg(feature = "wren-bt")]
    RNG => rng::InterruptHandler<RNG>;
    #[cfg(feature = "wren-bt")]
    EGU0_SWI0 => nrf_sdc::mpsl::LowPrioInterruptHandler;
    #[cfg(feature = "wren-bt")]
    CLOCK_POWER => nrf_sdc::mpsl::ClockInterruptHandler;
    #[cfg(feature = "wren-bt")]
    RADIO => nrf_sdc::mpsl::HighPrioInterruptHandler;
    #[cfg(feature = "wren-bt")]
    TIMER0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
    #[cfg(feature = "wren-bt")]
    RTC0 => nrf_sdc::mpsl::HighPrioInterruptHandler;
});

#[cfg(all(target_os = "none", feature = "wren-bt"))]
async fn init_bluetooth() {

}

#[cfg(all(target_os = "none", feature = "wren-bt"))]
#[cfg_attr(all(target_os = "none", feature = "wren-bt"), embassy_executor::main)]
async fn main(spawner: Spawner) {
    info!("Starting Wren-bt {}", env!("CARGO_PKG_VERSION"));
    info!("Enabling ext hfosc...");
    pac::CLOCK.tasks_hfclkstart().write_value(1);
    while pac::CLOCK.events_hfclkstarted().read() != 1 {}
    Timer::after_millis(100).await;

    let mut nrf_config = embassy_nrf::config::Config::default();
    nrf_config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    let p = embassy_nrf::init(nrf_config);


    // let mut pwr_pin = Output::new(p.P0_07, Level::High, OutputDrive::HighDrive);
    // let mut led_pin = Output::new(p.P0_07, Level::High, OutputDrive::HighDrive);
    let mut baro_cs = Output::new(p.P0_25, Level::High, OutputDrive::HighDrive);
    let acc_cs = Output::new(p.P0_10, Level::High, OutputDrive::HighDrive);
    let flash_cs = Output::new(p.P0_26, Level::High, OutputDrive::HighDrive);
    Timer::after_millis(100).await;

    // pwr_pin.set_high();
    baro_cs.set_high();
    Timer::after_millis(500).await;

    let mut spi_config = spim::Config::default();
    spi_config.frequency = spim::Frequency::M1;

    static SPI_BUS: StaticCell<Mutex<ThreadModeRawMutex, spim::Spim<peripherals::SPI2>>> = StaticCell::new();
    let spi = spim::Spim::new(p.SPI2, Irqs, p.P0_05, p.P0_04, p.P0_03, spi_config);

    let spi_bus = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);

    info!("initialising baro");
    let baro_spi_device: SpiDevice<'_, ThreadModeRawMutex, spim::Spim<'_, SPI2>, Output<'_>> =
        SpiDevice::new(spi_bus, baro_cs);
    let acc_spi_device = SpiDevice::new(spi_bus, acc_cs);
    let flash_spi_device = SpiDevice::new(spi_bus, flash_cs);

    let mut baro: BarometerType = bmp388::Bmp388::new(baro_spi_device, &mut Delay)
    .await
    .expect("initializing baro failed");
    info!("inbaro");
    baro.set_oversampling(bmp388::config::OversamplingConfig {
        osr_pressure: bmp388::Oversampling::x8,
        osr_temperature: bmp388::Oversampling::x1,
    })
    .await
    .expect("setting baro oversample failed");
    baro.set_sampling_rate(bmp388::SamplingRate::ms20)
        .await
        .expect("setting baro sample rate failed");
    baro.set_filter(bmp388::Filter::c1).await.expect("set baro filter failed");
    let power_control = bmp388::PowerControl {
        mode: bmp388::PowerMode::Normal,
        pressure_enable: true,
        temperature_enable: true,
    };
    baro.set_power_control(power_control).await.expect("couldn't set power control");



    #[embassy_executor::task]
    async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
        mpsl.run().await
    }

    fn build_sdc<'d, const N: usize>(
        p: nrf_sdc::Peripherals<'d>,
        rng: &'d mut rng::Rng<RNG>,
        mpsl: &'d MultiprotocolServiceLayer,
        mem: &'d mut sdc::Mem<N>,
    ) -> Result<nrf_sdc::SoftdeviceController<'d>, nrf_sdc::Error> {
        // How many outgoing L2CAP buffers per link
        const L2CAP_TXQ: u8 = 3;
        // How many incoming L2CAP buffers per link
        const L2CAP_RXQ: u8 = 3;
        // Size of L2CAP packets
        const L2CAP_MTU: usize = 27;

        sdc::Builder::new()?
            .support_adv()?
            .support_peripheral()?
            .peripheral_count(1)?
            .buffer_cfg(L2CAP_MTU as u8, L2CAP_MTU as u8, L2CAP_TXQ, L2CAP_RXQ)?
            .build(p, rng, mpsl, mem)
    }

    let mpsl_p = mpsl::Peripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let lfclk_cfg = mpsl::raw::mpsl_clock_lfclk_cfg_t {
        source: mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };
    static MPSL: StaticCell<MultiprotocolServiceLayer> = StaticCell::new();
    let mpsl = MPSL.init(mpsl::MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.must_spawn(mpsl_task(&*mpsl));

    let sdc_p = sdc::Peripherals::new(
        p.PPI_CH17, p.PPI_CH18, p.PPI_CH20, p.PPI_CH21, p.PPI_CH22, p.PPI_CH23, p.PPI_CH24, p.PPI_CH25, p.PPI_CH26,
        p.PPI_CH27, p.PPI_CH28, p.PPI_CH29,
    );

    let mut rng = rng::Rng::new(p.RNG, Irqs);

    let mut sdc_mem = sdc::Mem::<3312>::new();
    let sdc = build_sdc(sdc_p, &mut rng, mpsl, &mut sdc_mem).unwrap();

    loop {
        info!("boing");
        Timer::after_millis(1000).await;
    }
}

#[cfg(all(target_os = "none", feature = "wren"))]
#[cfg_attr(all(target_os = "none", feature = "wren"), embassy_executor::main)]
async fn main(spawner: Spawner) {
    let mut nrf_config = embassy_nrf::config::Config::default();
    nrf_config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    nrf_config.dcdc.reg0_voltage = Some(embassy_nrf::config::Reg0Voltage::_2V4); //flash memory is not rated for 1v8 apparently
    nrf_config.dcdc.reg1 = false;
    let p = embassy_nrf::init(nrf_config);

    info!("Starting Wren {}", env!("CARGO_PKG_VERSION"));

    let wdt_config = wdt::Config::try_new(&p.WDT).unwrap();
    let (_wdt, [mut wdt_handle]) = match wdt::Watchdog::try_new(p.WDT, wdt_config) {
        Ok(x) => x,
        Err(_) => {
            // Watchdog already active with the wrong number of handles, waiting for it to timeout...
            loop {
                cortex_m::asm::wfe();
            }
        }
    };


    info!("Enabling ext hfosc...");
    pac::CLOCK.tasks_hfclkstart().write_value(1);
    while pac::CLOCK.events_hfclkstarted().read() != 1 {}
    Timer::after_millis(100).await;

    let mut led_pin = Output::new(p.P0_09, Level::High, OutputDrive::HighDrive);
    let mut pwr_pin = Output::new(p.P0_01, Level::High, OutputDrive::HighDrive);
    let baro_cs = Output::new(p.P0_30, Level::High, OutputDrive::HighDrive);
    let acc_cs = Output::new(p.P0_03, Level::High, OutputDrive::HighDrive);
    let flash_cs = Output::new(p.P0_10, Level::High, OutputDrive::HighDrive);

    pwr_pin.set_high();
    led_pin.set_high();
    Timer::after_millis(500).await;

    let mut spi_config = spim::Config::default();
    spi_config.frequency = spim::Frequency::M1;

    static SPI_BUS: StaticCell<Mutex<ThreadModeRawMutex, spim::Spim<peripherals::SPI2>>> = StaticCell::new();
    let spi = spim::Spim::new(p.SPI2, Irqs, p.P0_29, p.P0_02, p.P0_28, spi_config);

    let spi_bus = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);

    let baro_spi_device: SpiDevice<'_, ThreadModeRawMutex, spim::Spim<'_, SPI2>, Output<'_>> =
        SpiDevice::new(spi_bus, baro_cs);
    let acc_spi_device = SpiDevice::new(spi_bus, acc_cs);
    let flash_spi_device = SpiDevice::new(spi_bus, flash_cs);

    let usb_driver = usb::Driver::new(p.USBD, Irqs, usb::vbus_detect::HardwareVbusDetect::new(Irqs));

    let flash = FlashMemoryType::new(flash_spi_device, DummyPin::new_low(), DummyPin::new_low()).unwrap();
    // flash.erase_all().await.expect("couldn't erase flash");

    let mut baro: BarometerType = bmp388::Bmp388::new(baro_spi_device, &mut Delay)
        .await
        .expect("initializing baro failed");
    baro.set_oversampling(bmp388::config::OversamplingConfig {
        osr_pressure: bmp388::Oversampling::x8,
        osr_temperature: bmp388::Oversampling::x1,
    })
    .await
    .expect("setting baro oversample failed");
    baro.set_sampling_rate(bmp388::SamplingRate::ms20)
        .await
        .expect("setting baro sample rate failed");
    baro.set_filter(bmp388::Filter::c1).await.expect("set baro filter failed");
    let power_control = bmp388::PowerControl {
        mode: bmp388::PowerMode::Normal,
        pressure_enable: true,
        temperature_enable: true,
    };
    baro.set_power_control(power_control).await.expect("couldn't set power control");

    let mut acc = AccelerometerType::new(acc_spi_device).await.expect("couldn't initialize acc");
    acc.set_bdu(true).await.expect("coudln't set bdu");
    acc.enabled_adcs().await.expect("coudln't enable adc");
    acc.enable_temp(true).await.expect("couldn't enable temp");
    acc.enable_axis((true, true, true)).await.expect("couldn't set acc axis");
    acc.set_mode(lis2dh12::Mode::HighResolution)
        .await
        .expect("couldn't set acc mode");
    acc.set_fs(lis2dh12::FullScale::G16).await.expect("couldn't set acc scale");
    acc.set_odr(lis2dh12::Odr::Hz100).await.expect("couldn't set acc ODR");

    static FLASH: StaticCell<Mutex<ThreadModeRawMutex, FlashWriter>> = StaticCell::new();
    let flash = FLASH.init(Mutex::new(FlashWriter::new(flash).await.unwrap()));

    static WREN: StaticCell<Mutex<ThreadModeRawMutex, WrenState>> = StaticCell::new();
    let wren = WrenState {
        altitude: 0.0,
        acceleration: 0.0,
        battery_voltage: 0.0,
    };
    let wren = WREN.init(Mutex::new(wren));

    spawner.spawn(flight_task::flight_task(baro, acc, flash, wren)).unwrap();

    spawner.spawn(usb_handler::usb_task(spawner, usb_driver, flash, wren)).unwrap();

    let mut adc_config = saadc::Config::default();
    adc_config.oversample = saadc::Oversample::OVER256X;
    adc_config.resolution = saadc::Resolution::_14BIT;
    let mut channel_config = saadc::ChannelConfig::single_ended(p.P0_04);
    channel_config.reference = saadc::Reference::INTERNAL; // 0.6V
    channel_config.gain = saadc::Gain::GAIN1_3; // 0.6V * 3 = 1.8V
    channel_config.time = saadc::Time::_40US; // high input resistance in the divider
    let saadc: saadc::Saadc<'_, 1> = saadc::Saadc::new(p.SAADC, Irqs, adc_config, [channel_config]);
    saadc.calibrate().await;

    #[embassy_executor::task]
    async fn adc_task(mut saadc: saadc::Saadc<'static, 1>, wren: &'static Mutex<ThreadModeRawMutex, WrenState>) {
        loop {
            let mut adc_buf = [0_i16; 1];
            saadc.sample(&mut adc_buf).await;
            let volt = adc_buf[0] as f32 / (powf(2.0, 14.0) - 1.0) * 1.8 * 151.0 / 51.0;
            {
                let mut wren = wren.lock().await;
                wren.battery_voltage = volt;
            };
            Timer::after_millis(250).await;
        }
    }
    spawner.spawn(adc_task(saadc, wren)).unwrap();

    let mut index = flash.lock().await.get_index();
    let mut size = flash.lock().await.get_size();
    loop {
        match with_timeout(embassy_time::Duration::from_millis(100), flash.lock()).await {
            Ok(flash) => {
                index = flash.get_index();
                size = flash.get_size();
            },
            Err(_) => {}
        }
        if index < 1 {
            index = 1
        }

        let to_blink = 4 - (((index - 1) as f32 / size as f32) * 4.0) as u32;
        for _ in 0..to_blink {
            led_pin.set_high();
            Timer::after_millis(200).await;
            led_pin.set_low();
            Timer::after_millis(200).await;
        }
        Timer::after_millis(1000).await;
        wdt_handle.pet();
    }
}

#[cfg(not(target_os = "none"))]
#[embassy_executor::task]
async fn init_native(spawner: Spawner) {
    let baro = BarometerType::new();
    let accel = AccelerometerType::new();
    let flash = FlashMemoryType::new();

    static FLASH: StaticCell<Mutex<ThreadModeRawMutex, FlashWriter>> = StaticCell::new();
    let flash = FLASH.init(Mutex::new(FlashWriter::new(flash).await.unwrap()));

    static WREN: StaticCell<Mutex<ThreadModeRawMutex, WrenState>> = StaticCell::new();
    let wren = WrenState {
        altitude: 0.0,
        acceleration: 0.0,
        battery_voltage: 0.0,
    };
    let wren = WREN.init(Mutex::new(wren));
    spawner.spawn(flight_task::flight_task(baro, accel, flash, wren)).unwrap();
    loop {
        Timer::after_millis(500).await;
    }
}

#[cfg(not(target_os = "none"))]
fn main() {
    env_logger::init();
    use embassy_executor::Executor;
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(init_native(spawner)).unwrap();
    });
}
