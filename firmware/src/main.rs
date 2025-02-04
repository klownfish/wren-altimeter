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
use embassy_nrf::{bind_interrupts, peripherals, spim, usb, wdt};
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

#[cfg(target_os = "none")]
mod usb_handler;

#[allow(unused)]
struct WrenState {
    battery_voltage: f32,
    acceleration: f32,
    altitude: f32,
}

#[cfg(target_os = "none")]
bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    SPI2 => spim::InterruptHandler<SPI2>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
    SAADC => saadc::InterruptHandler;
});

#[cfg(target_os = "none")]
#[embassy_executor::main]
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
    pwr_pin.set_high();
    led_pin.set_high();
    Timer::after_millis(500).await;

    let mut spi_config = spim::Config::default();
    spi_config.frequency = spim::Frequency::M1;

    static SPI_BUS: StaticCell<Mutex<ThreadModeRawMutex, spim::Spim<peripherals::SPI2>>> = StaticCell::new();
    let spi = spim::Spim::new(p.SPI2, Irqs, p.P0_29, p.P0_02, p.P0_28, spi_config);

    let spi_bus = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);

    let baro_cs = Output::new(p.P0_30, Level::High, OutputDrive::HighDrive);
    let acc_cs = Output::new(p.P0_03, Level::High, OutputDrive::HighDrive);
    let flash_cs = Output::new(p.P0_10, Level::High, OutputDrive::HighDrive);

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
    error!("haha");
    use embassy_executor::Executor;
    static EXECUTOR: StaticCell<Executor> = StaticCell::new();
    let executor = EXECUTOR.init(Executor::new());
    executor.run(|spawner| {
        spawner.spawn(init_native(spawner)).unwrap();
    });
}
