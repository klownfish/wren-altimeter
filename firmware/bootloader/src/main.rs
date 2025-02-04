#![no_std]
#![no_main]

use core::cell::RefCell;

use cortex_m_rt::{entry, exception};
use defmt_rtt as _;
use embassy_boot_nrf::*;
use embassy_nrf::nvmc::{self, Nvmc};
use embassy_nrf::wdt::{self, HaltConfig, SleepConfig};
use embassy_nrf::{bind_interrupts, peripherals, usb, pac};
use embassy_executor::Spawner;
use embassy_time::{with_timeout, Duration};
use embassy_sync::blocking_mutex::Mutex;
use defmt::info;
use static_cell::StaticCell;
use embassy_nrf::gpio::{Output, Level, OutputDrive};

use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config, UsbDevice};
use embassy_nrf::usb::vbus_detect::HardwareVbusDetect;

bind_interrupts!(struct Irqs {
    USBD => usb::InterruptHandler<peripherals::USBD>;
    CLOCK_POWER => usb::vbus_detect::InterruptHandler;
});

type UsbClass<'a> = CdcAcmClass<'a, usb::Driver<'a, peripherals::USBD, HardwareVbusDetect>>;

pub fn app_start() -> u32 {
    extern "C" {
        static __application_start: u32;
    }
    unsafe { &__application_start as *const u32 as u32 }
}

pub fn app_end() -> u32 {
    extern "C" {
        static __application_end: u32;
    }
    unsafe { &__application_end as *const u32 as u32 }
}

#[embassy_executor::task]
async fn usb_driver_task(
    mut usb: UsbDevice<'static, embassy_nrf::usb::Driver<'static, peripherals::USBD, HardwareVbusDetect>>,
) {
    usb.run().await;
}

async fn handle_usb_connection(
    mut flash: WatchdogFlash<Nvmc<'_>>,
    mut class: UsbClass<'_>
) -> Result<(), EndpointError> {

    let mut rx_buf = [0_u8; 1024];
    let mut tx_buf = [0_u8; 64];
    info!("reading packet");
    let fut = class.read_packet(&mut rx_buf);
    let len = match with_timeout(Duration::from_millis(4000), fut).await {
        Err(_) => {
            info!("got no packet");
            return Ok(())
        },
        Ok(res) => res?
    };
    info!("read packet");
    if len != 10 {
        info!("bad length {}", len);
        return Ok(());
    }
    if &rx_buf[0..6] != b"UPDATE" {
        info!("bad header {}", &rx_buf[0..6]);
        return Ok(());
    }
    let length = u32::from_le_bytes(rx_buf[6..10].try_into().expect("impossible"));
    info!("erasing flash");
    flash.erase(app_start(), app_start() + length + nvmc::PAGE_SIZE as u32 - length % nvmc::PAGE_SIZE as u32).expect("Could not erase the app flash partition");

    info!("starting write");
    let mut index = 0;
    while index < length {
        let len = class.read_packet(&mut rx_buf).await?;
        flash.write(app_start() + index, &rx_buf[0..len]).expect("Writing application failed");
        index += len as u32;
    }

    Ok(())
}


#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {
    let mut nrf_config = embassy_nrf::config::Config::default();
    nrf_config.hfclk_source = embassy_nrf::config::HfclkSource::ExternalXtal;
    let p = embassy_nrf::init(nrf_config);
    let mut led_pin = Output::new(p.P0_09, Level::High, OutputDrive::HighDrive);
    led_pin.set_high();
    // Uncomment this if you are debugging the bootloader with debugger/RTT attached,
    // as it prevents a hard fault when accessing flash 'too early' after boot.
    for _ in 0..100000 {
        cortex_m::asm::nop();
    }
    info!("Entered bootloader");
    let mut wdt_config = wdt::Config::default();
    wdt_config.timeout_ticks = 32768 * 10; // timeout seconds
    wdt_config.action_during_sleep = SleepConfig::RUN;
    wdt_config.action_during_debug_halt = HaltConfig::PAUSE;
    let flash = WatchdogFlash::start(Nvmc::new(p.NVMC), p.WDT, wdt_config);

    info!("Enabling ext hfosc...");
    pac::CLOCK.tasks_hfclkstart().write_value(1);
    while pac::CLOCK.events_hfclkstarted().read() != 1 {}

    // buffer size not applicable
    let bl = BootLoader::<4096>{};

    let usb_driver = usb::Driver::new(p.USBD, Irqs, usb::vbus_detect::HardwareVbusDetect::new(Irqs));

    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Klownfish");
    config.product = Some("Wren-Bootloader");
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
        usb_driver,
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        MSOS_DESCRIPTOR.init([0; 256]),
        CONTROL_BUF.init([0; 64]),
    );

    static STATE: StaticCell<State> = StaticCell::new();
    let state = STATE.init(State::new());

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, state, 512);

    // Build the builder.
    let usb = builder.build();

    spawner.spawn(usb_driver_task(usb)).unwrap();

    info!("waiting for connection");
    let wait_fut = class.wait_connection();
    match with_timeout(Duration::from_millis(2000), wait_fut).await {
        Err(_) => {info!("no connection");},
        Ok(_) => {
            info!("got connection");
            let _ = handle_usb_connection(flash, class).await;
        }
    }

    info!("Loading Application");

    // short break to let debug messages out
    for _ in 0..1000000 {
        cortex_m::asm::nop();
    }

    unsafe { bl.load(app_start()) }
}

#[no_mangle]
#[cfg_attr(target_os = "none", link_section = ".HardFault.user")]
unsafe extern "C" fn HardFault() {
    cortex_m::peripheral::SCB::sys_reset();
}

#[exception]
unsafe fn DefaultHandler(_: i16) -> ! {
    const SCB_ICSR: *const u32 = 0xE000_ED04 as *const u32;
    let irqn = core::ptr::read_volatile(SCB_ICSR) as u8 as i16 - 16;

    panic!("DefaultHandler #{:?}", irqn);
}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    cortex_m::asm::udf();
}