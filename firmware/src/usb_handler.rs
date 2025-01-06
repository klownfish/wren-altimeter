use core::fmt::Write;

#[allow(unused_imports)]
use defmt::{debug, error, info, warn};
use embassy_executor::Spawner;
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;
use embassy_nrf::{peripherals, usb};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Instant;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config, UsbDevice};
use static_cell::StaticCell;

use super::flash_writer::FlashWriter;

#[embassy_executor::task]
async fn usb_driver_task(
    mut usb: UsbDevice<'static, embassy_nrf::usb::Driver<'static, peripherals::USBD, HardwareVbusDetect>>,
) {
    loop {
        usb.run().await;
    }
}

#[embassy_executor::task]
pub async fn usb_task(
    spawner: Spawner,
    driver: usb::Driver<'static, peripherals::USBD, HardwareVbusDetect>,
    flash: &'static Mutex<ThreadModeRawMutex, FlashWriter>,
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
    let mut class = CdcAcmClass::new(&mut builder, state, 64);

    // Build the builder.
    let usb = builder.build();

    spawner.spawn(usb_driver_task(usb)).unwrap();
    let mut rx_buf = [0_u8; 512];
    let mut command_buf = [0_u8; 128];
    let mut command_index: usize = 0;
    loop {
        class.wait_connection().await;
        loop {
            if let Ok(len) = class.read_packet(&mut rx_buf).await {
                let slice = &mut rx_buf[0..len];
                for c in slice {
                    match c {
                        b'\r' => continue,
                        b'\n' => {
                            handle_command(&mut class, &command_buf, command_index, flash).await;
                            command_index = 0
                        }
                        _ => {
                            command_buf[command_index] = *c;
                            command_index += 1
                        }
                    }
                }
            } else {
                break;
            }
        }
    }
}

async fn handle_command<'a>(
    class: &mut CdcAcmClass<'a, usb::Driver<'a, peripherals::USBD, HardwareVbusDetect>>,
    buf: &[u8],
    length: usize,
    flash: &'static Mutex<ThreadModeRawMutex, FlashWriter>,
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
            write!(
                str_buf,
                "{{'used_flash':{}, 'flash_size':{}, 'time':{}, 'volt':{}}}",
                flash.get_index(),
                flash.get_size(),
                Instant::now().as_millis(),
                3.8
            )
            .unwrap();
            class.write_packet(str_buf.as_bytes()).await.unwrap();
        }

        _ => {
            info!("invalid command");
        }
    }
}
