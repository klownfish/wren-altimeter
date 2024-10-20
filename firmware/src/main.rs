#![no_std]
#![no_main]


use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::{bind_interrupts, peripherals, spim, usb, pac};

use embassy_nrf::usb::vbus_detect::{HardwareVbusDetect, VbusDetect};
use embassy_nrf::usb::{Driver, Instance};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};

use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use static_cell::StaticCell;

use bmp388;
use lis2dh12;

use embassy_time::Timer;
use embassy_time::Delay;
use w25q32jv::W25q32jv;
use {defmt_rtt as _, panic_probe as _};

#[allow(unused_imports)]
use defmt::*;

use dummy_pin::DummyPin;

mod kalman;

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    SPIM3 => spim::InterruptHandler<peripherals::SPI3>;
    POWER_CLOCK => usb::vbus_detect::InterruptHandler;
});

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut nrf_config = embassy_nrf::config::Config::default();
    nrf_config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    let p = embassy_nrf::init(Default::default());

    // why do they enable hclk manually in the examples lol
    // let clock: pac::CLOCK = unsafe { mem::transmute(()) };

    // info!("Enabling ext hfosc...");
    // clock.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });
    // while clock.events_hfclkstarted.read().bits() != 1 {}

    let mut spi_config = spim::Config::default();
    spi_config.frequency = spim::Frequency::M1;

    static SPI_BUS: StaticCell<Mutex<NoopRawMutex, spim::Spim<peripherals::SPI3>>> = StaticCell::new();
    let spi = spim::Spim::new(p.SPI3, Irqs, p.P0_29, p.P0_02, p.P0_28, spi_config);
    let spi_bus = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);

    let baro_cs = Output::new(p.P0_30, Level::High, OutputDrive::HighDrive);
    let acc_cs = Output::new(p.P0_03, Level::High, OutputDrive::HighDrive);
    let flash_cs = Output::new(p.P0_10, Level::High, OutputDrive::HighDrive);

    let baro_spi_device = SpiDevice::new(spi_bus, baro_cs);
    let acc_spi_device = SpiDevice::new(spi_bus, acc_cs);
    let flash_spi_device = SpiDevice::new(spi_bus, flash_cs);

    let mut baro = bmp388::Bmp388::new(baro_spi_device, &mut Delay).await.expect("baro failed");
    baro.set_sampling_rate(bmp388::SamplingRate::ms10).await.unwrap();

    let mut acc = lis2dh12::Lis2dh12::new(acc_spi_device).await.expect("baro failed");
    acc.enable_axis((true, true, true)).await.unwrap();

    // should probably update the driver instead of using dummy pin but whatever
    let mut flash = W25q32jv::new(flash_spi_device, DummyPin::new_low(), DummyPin::new_low()).unwrap();
    flash.device_id_async().await.expect("flash failed");

    let mut kalman = kalman::Kalman::<3>::new();
    let a = kalman::AMatrix::<3>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    let q = kalman::AMatrix::<3>::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    kalman.update_process(&q, &a);

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USBD, Irqs, HardwareVbusDetect::new(Irqs));

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = true;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut msos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut msos_descriptor,
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    loop {
        Timer::after_millis(300).await;
    }
}