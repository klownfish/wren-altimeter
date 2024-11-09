#![no_std]
#![no_main]

use core::fmt::Write;

use embassy_nrf::gpio::{Output, OutputDrive, Level};
use embassy_nrf::{bind_interrupts, peripherals, spim, usb};
use embassy_nrf::usb::vbus_detect::{HardwareVbusDetect};
use embassy_nrf::peripherals::SPI2;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config, UsbDevice};
use flight_sm::{FlightSM, FlightState};
use static_cell::StaticCell;
use embassy_executor::Spawner;
use embassy_time::{Timer, Instant};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use nalgebra::Vector3;
use flash_stuff::FlashStuff;

use libm::{pow};

use bmp388;
use lis2dh12;
use embassy_time::Delay;
use w25q32jv::W25q32jv;
use {defmt_rtt as _, panic_probe as _};

#[allow(unused_imports)]
use defmt::{info, error, warn, debug};

mod kalman;
mod flight_sm;
mod flash_stuff;

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    SPIM2_SPIS2_SPI2 => spim::InterruptHandler<SPI2>;
    POWER_CLOCK => usb::vbus_detect::InterruptHandler;
});

type Spi2Device = SpiDevice<'static, ThreadModeRawMutex, spim::Spim<'static, SPI2>, Output<'static>>;

#[embassy_executor::task]
async fn flight_task(
    baro_device: Spi2Device,
    acc_device: Spi2Device,
    flash: &'static Mutex<ThreadModeRawMutex, flash_stuff::FlashStuff<Spi2Device>>
) -> ! {

    let mut baro = bmp388::Bmp388::new(baro_device, &mut Delay).await.expect("initializing baro failed");
    baro.set_oversampling(bmp388::config::OversamplingConfig { osr_pressure: bmp388::Oversampling::x8, osr_temperature: bmp388::Oversampling::x1}).await.expect("setting baro oversample failed");
    baro.set_sampling_rate(bmp388::SamplingRate::ms20).await.expect("setting baro sample rate failed");
    baro.set_filter(bmp388::Filter::c1).await.expect("set baro filter failed");

    let mut acc = lis2dh12::Lis2dh12::new(acc_device).await.expect("couldn't initialize acc");
    acc.enable_axis((true, true, true)).await.expect("couldn't set acc axis");
    acc.set_mode(lis2dh12::Mode::HighResolution).await.expect("couldn't set acc mode");
    acc.set_fs(lis2dh12::FullScale::G16).await.expect("couldn't set acc scale");
    acc.set_odr(lis2dh12::Odr::Hz100).await.expect("couldn't set acc ODR");

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
        let to_sleep = Instant::now().as_millis() % 50;
        Timer::after_millis(to_sleep).await;

        // get measurements
        let altitude = match baro.sensor_values().await {
            Ok(val) => (288.15_f64 / 0.0065_f64 * (pow(val.pressure / 101325_f64, -0.1903_f64) - 1_f64)) as f32,
            Err(_) => {
                error!("Could not get baro measurement");
                continue;
            }
        };

        let acceleration = match acc.accel_norm().await {
            Ok(val) => Vector3::<f32>::new(val[0], val[1], val[2]),
            Err(_) => {
                error!("Could not get accel measurement");
                continue;
            }
        };
        cycle_count += 1;

        let old_state = flight_sm.get_state();
        flight_sm.update(altitude, acceleration);
        let new_state = flight_sm.get_state();

        if flight_sm.get_altitude() > max_altitude {
            max_altitude = flight_sm.get_altitude();
        }

        let write_every = match flight_sm.get_state() {
            FlightState::Boost => 1,
            FlightState::Idle => 50,
            _ => 10,
        };

        if cycle_count % write_every == 0 {
            flash.lock().await.write_telem(0.0, 0.0, 0.0).await;
        }

        if cycle_count % (write_every * 10) == 0 {
            flash.lock().await.write_metadata(Instant::now(), 3.3).await;
        }

        if old_state != new_state {
            flash.lock().await.write_state(new_state).await;
        }
    }
}

#[embassy_executor::task]
async fn usb_driver_task(
    mut usb: UsbDevice<'static, embassy_nrf::usb::Driver<'static, peripherals::USBD, HardwareVbusDetect>>
) {
    loop {
        usb.run().await;
    }
}

#[embassy_executor::task]
async fn usb_task(
    spawner: Spawner,
    driver: usb::Driver<'static, peripherals::USBD, HardwareVbusDetect>,
    flash: &'static Mutex<ThreadModeRawMutex, flash_stuff::FlashStuff<Spi2Device>>
) {
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
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static MSOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    let mut builder = Builder::new(
        driver,
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        MSOS_DESCRIPTOR.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );

    static STATE: StaticCell<State> = StaticCell::new();
    let state = STATE.init(State::new());

    // Create classes on the builder.
    let mut class: CdcAcmClass<'_, usb::Driver<'_, peripherals::USBD, HardwareVbusDetect>> = CdcAcmClass::new(&mut builder, state, 64);

    // Build the builder.
    let usb: embassy_usb::UsbDevice<'_, usb::Driver<'_, peripherals::USBD, HardwareVbusDetect>> = builder.build();

    spawner.spawn(usb_driver_task(usb)).unwrap();
    let mut rx_buf = [0_u8; 512];
    let mut command_buf = [0_u8; 128];
    let mut command_index: usize = 0;
    loop {
        if let Ok(len) = class.read_packet(&mut rx_buf).await {
            let slice = &mut rx_buf[0..len];
            for c in slice {
                match c {
                    b'\r' => continue,
                    b'\n' =>  {
                        handle_command(&mut class, &command_buf, command_index, flash).await;
                        command_index = 0
                    }
                    _ => {
                        command_buf[command_index] = *c;
                        command_index += 1
                    }
                }
            }
        }

    }
}

async fn handle_command<'a>(
    class: &mut CdcAcmClass<'a, usb::Driver<'a, peripherals::USBD, HardwareVbusDetect>>,
    buf: &[u8], length: usize, flash: &'static Mutex<ThreadModeRawMutex,
    flash_stuff::FlashStuff<Spi2Device>>
) {
    let cmd = &buf[0..length];

    match cmd {
        b"read_flash" => {
            let mut str_buf: heapless::String<64> = Default::default();
            let mut flash = flash.lock().await;
            write!(str_buf, "{{'length':{}}}", flash.get_index()).unwrap();
            class.write_packet(str_buf.as_bytes()).await.unwrap();
            let mut read: u32 = 0;
            let mut buf = [0_u8; 256];
            while read < flash.get_index() {
                let to_read = (flash.get_index() - read).min(buf.len() as u32) as usize;
                let slice = &mut buf[0..to_read];
                flash.read(read, slice).await.unwrap();
                class.write_packet(slice).await.unwrap();
                read += to_read as u32;
            }
        }

        b"clear_flash" => {
            flash.lock().await.erase().await;
        }

        b"get_status" => {
            let flash = flash.lock().await;
            let mut str_buf: heapless::String<128> = Default::default();
            write!(str_buf,
                "{{'used_flash':{}, 'flash_size':{}, 'time':{}, 'volt':{}}}",
                flash.get_index(), flash.get_size(), Instant::now().as_millis(), 3.8
            ).unwrap();
            class.write_packet(str_buf.as_bytes()).await.unwrap();
        }

        _ => {
            info!("invalid command");
        }
    }
}

#[allow(unused)]
async fn set_regulator_voltage() {
    let target = 0;
    unsafe {
        let nvmc_base = 0x4001E000;
        let p_config: *mut u32 = (nvmc_base + 0x504) as *mut u32;

        let uicr_base = 0x10001000;
        let p_uicr_regout0: *mut u32 = (uicr_base + 0x304) as *mut u32;

        let p_erase_uicr: *mut u32 = (nvmc_base + 0x514) as *mut u32;

        core::ptr::write_volatile(p_config, 0 as u32); // enable flash read
        let before = core::ptr::read_volatile(p_uicr_regout0);

        if before | 0b111 != target {
            core::ptr::write_volatile(p_config, 2 as u32); // enable flash erase
            core::ptr::write_volatile(p_erase_uicr, 1 as u32); // erase uicr
            let middle = core::ptr::read_volatile(p_uicr_regout0);
            core::ptr::write_volatile(p_config, 1 as u32); // enable flash writing
            core::ptr::write_volatile(p_uicr_regout0, target as u32); // set to target voltage
            core::ptr::write_volatile(p_config, 0 as u32); // disable uicr writing
            let after = core::ptr::read_volatile(p_uicr_regout0);
            info!("Set regout0 before: {} middle: {} after: {}", before, middle, after);
            Timer::after_millis(500).await;

        } else {
            info!("reg0 already set to {}", target);
            cortex_m::peripheral::SCB::sys_reset();
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
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
    spi_config.frequency = spim::Frequency::M1;

    static SPI_BUS: StaticCell<Mutex<ThreadModeRawMutex, spim::Spim<peripherals::SPI2>>> = StaticCell::new();
    let spi = spim::Spim::new(p.SPI2, Irqs, p.P0_29, p.P0_02, p.P0_28, spi_config);

    let spi_bus = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);

    let baro_cs = Output::new(p.P0_30, Level::High, OutputDrive::HighDrive);
    let acc_cs = Output::new(p.P0_03, Level::High, OutputDrive::HighDrive);
    let flash_cs = Output::new(p.P0_10, Level::High, OutputDrive::HighDrive);

    let baro_spi_device = SpiDevice::new(spi_bus, baro_cs);
    let acc_spi_device = SpiDevice::new(spi_bus, acc_cs);
    let flash_spi_device = SpiDevice::new(spi_bus, flash_cs);

    let usb_driver = usb::Driver::new(p.USBD, Irqs, usb::vbus_detect::HardwareVbusDetect::new(Irqs));

    static FLASH: StaticCell<Mutex<ThreadModeRawMutex, FlashStuff<Spi2Device>>> = StaticCell::new();
    let flash = FLASH.init(Mutex::new(FlashStuff::new(flash_spi_device).await.unwrap()));

    // spawner.spawn(flash_task()).unwrap();

    spawner.spawn(flight_task(baro_spi_device, acc_spi_device, flash)).unwrap();
    spawner.spawn(usb_task(spawner, usb_driver, flash)).unwrap();
    loop {
        led_pin.toggle();
        Timer::after_millis(500).await;
    };
}