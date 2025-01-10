#![cfg_attr(target_os = "none", no_std)]
#![cfg_attr(target_os = "none", no_main)]

#[allow(unused_imports)]
#[cfg(target_os = "none")]
use defmt::{debug, error, info, trace, warn};
#[cfg(target_os = "none")]
use dummy_pin::DummyPin;
#[cfg(target_os = "none")]
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_executor::Spawner;
#[cfg(target_os = "none")]
use embassy_nrf::gpio::{Level, Output, OutputDrive};
#[cfg(target_os = "none")]
use embassy_nrf::peripherals::SPI2;
#[cfg(target_os = "none")]
use embassy_nrf::{bind_interrupts, peripherals, spim, usb};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
#[cfg(target_os = "none")]
use embassy_time::Delay;
use embassy_time::{Instant, Timer};
#[cfg(not(target_os = "none"))]
use env_logger;
use flash_writer::FlashWriter;
use flight_sm::{FlightSM, FlightState};
#[allow(unused_imports)]
#[cfg(not(target_os = "none"))]
use log::{debug, error, info, trace, warn};
use nalgebra::Vector3;
use platform::{Accelerometer, AccelerometerType, Barometer, BarometerType, FlashMemoryType};
use static_cell::StaticCell;
#[cfg(target_os = "none")]
use {defmt_rtt as _, panic_probe as _};

mod flash_writer;
mod flight_sm;
mod kalman;
mod platform;

#[cfg(target_os = "none")]
mod usb_handler;

#[cfg(target_os = "none")]
bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    SPI2 => spim::InterruptHandler<SPI2>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
});

#[embassy_executor::task]
async fn flight_task(
    mut baro: BarometerType,
    mut acc: AccelerometerType,
    flash: &'static Mutex<ThreadModeRawMutex, flash_writer::FlashWriter>,
) -> ! {
    // write multiple times to guarante a proper sync if the previous state is corrupt
    for _ in 0..16 {
        flash.lock().await.write_power_on().await;
    }
    flash.lock().await.write_metadata(Instant::now(), 0.0).await;
    flash.lock().await.write_state(FlightState::Idle).await;

    let mut flight_sm = FlightSM::new();

    let mut max_altitude = 0_f32;
    let mut cycle_count = 0_u32;
    loop {
        Timer::after_millis(20).await; // make this stable 50Hz
                                       // get measurements
        let baro_reading = match baro.read_temp_and_pressure().await {
            Ok(val) => val,
            Err(_) => {
                error!("Could not get baro measurement");
                continue;
            }
        };

        let acceleration = match acc.read_acceleration().await {
            Ok(val) => Vector3::<f32>::new(val[0], val[1], val[2]),
            Err(_) => {
                error!("Could not get accel measurement");
                continue;
            }
        };
        cycle_count += 1;

        let old_state = flight_sm.get_state();
        flight_sm.update(baro_reading.pressure, acceleration);
        let new_state = flight_sm.get_state();

        if flight_sm.get_relative_altitude() > max_altitude {
            max_altitude = flight_sm.get_relative_altitude();
        }

        let write_every = match flight_sm.get_state() {
            FlightState::MaybeLaunched => 1,
            FlightState::Boost => 1,
            FlightState::Coast => 1,
            FlightState::Descent => 10,
            FlightState::Idle => 50,
        };

        let mut flash = flash.lock().await;

        if cycle_count % write_every == 0 {
            flash
                .write_telem(
                    flight_sm.get_relative_altitude(),
                    flight_sm.get_vertical_velocity(),
                    flight_sm.get_vertical_acceleration(),
                )
                .await;
            info!(
                "telemetry {} {} {}",
                flight_sm.get_relative_altitude(),
                flight_sm.get_vertical_velocity(),
                flight_sm.get_vertical_acceleration()
            );
            info!("flash {}", flash.get_index());
        }

        if cycle_count % (write_every * 10) == 0 {
            flash.write_metadata(Instant::now(), 3.3).await;
        }

        #[cfg(feature = "store_all")]
        if flight_sm.get_state() != FlightState::Idle {
            flash
                .write_debug(
                    acceleration[0],
                    acceleration[1],
                    acceleration[2],
                    baro_reading.pressure,
                    Instant::now(),
                )
                .await;
        }
        if old_state != new_state {
            warn!("state changed {:?}", new_state);
            flash.write_state(new_state).await;
        }

        trace!(
            "debug {:?} {} {} {}",
            new_state,
            flight_sm.get_absolute_altitude(),
            flight_sm.get_vertical_velocity(),
            flight_sm.get_vertical_acceleration()
        );
        trace!(
            "debug raw {} {} {} {}",
            baro_reading.pressure,
            acceleration[0],
            acceleration[1],
            acceleration[2]
        );
        trace!("debug flash {}", flash.get_index());
    }
}

#[cfg(target_os = "none")]
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut nrf_config = embassy_nrf::config::Config::default();
    nrf_config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    let p = embassy_nrf::init(Default::default());

    // set_regulator_voltage().await;
    let mut led_pin = Output::new(p.P0_09, Level::High, OutputDrive::HighDrive);

    let mut pwr_pin = Output::new(p.P0_01, Level::High, OutputDrive::HighDrive);
    pwr_pin.set_high();
    led_pin.set_high();
    Timer::after_millis(500).await;

    let mut spi_config = spim::Config::default();
    spi_config.frequency = spim::Frequency::M8;

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
    let mut acc = lis2dh12::Lis2dh12::new(acc_spi_device).await.expect("couldn't initialize acc");
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

    spawner.spawn(flight_task(baro, acc, flash)).unwrap();
    spawner.spawn(usb_handler::usb_task(spawner, usb_driver, flash)).unwrap();
    loop {
        led_pin.toggle();
        Timer::after_millis(500).await;
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

    spawner.spawn(flight_task(baro, accel, flash)).unwrap();
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
