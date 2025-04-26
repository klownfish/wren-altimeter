use core::ops::Deref;

use embassy_futures::join::join;
use embassy_futures::select::select;
use embassy_time::Timer;
use heapless::Vec;
use trouble_host::prelude::{Controller, gatt_server, gatt_service, service, descriptors, characteristic, Address, HostResources, GapConfig, PeripheralConfig, appearance, Host, Runner, Connection, BleHostError, AdStructure, ConnectionEvent, GattEvent, Peripheral, LE_GENERAL_DISCOVERABLE, BR_EDR_NOT_SUPPORTED, Uuid, Advertisement, Stack};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use crate::nus::{GattService, NusService, NusManager};

#[allow(unused)]
use defmt::{debug, info, warn, error};

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

// GATT Server definition
#[gatt_server]
struct Server {
    nus_service: NusService,
}

type NusType<'a> = NusManager<'a, CriticalSectionRawMutex, 20, 200, 200>;

/// Run the BLE stack.
pub async fn run<C, const L2CAP_MTU: usize>(controller: C)
where
    C: Controller,
{
    // Using a fixed "random" address can be useful for testing. In real scenarios, one would
    // use e.g. the MAC 6 byte array as the address (how to get that varies by the platform).
    let address: Address = Address::random([0xff, 0x8f, 0x1a, 0x05, 0xe4, 0xff]);
    info!("Our address = {:?}", address);

    let mut resources: HostResources<CONNECTIONS_MAX, L2CAP_CHANNELS_MAX, L2CAP_MTU> = HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral, runner, ..
    } = stack.build();



    info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "Wren",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();


    let nus = NusManager::new(&server.nus_service.rx, &server.nus_service.tx);
    async fn loopback_task(nus: &NusType<'_>) {
        loop {
            let mut buf = [0_u8; 255];
            info!("waiting for message");
            let len = nus.receive_message(&mut buf).await;
            info!("received message");
            nus.send_message(&mut buf[0..len]).await;
            info!("sent message");
        }
    }


    let _ = join(ble_task(runner), async {
        loop {
            match advertise("Wren", &mut peripheral).await {
                Ok(conn) => {
                    // set up tasks when the connection is established to a central, so they don't run when no one is connected.
                    let a = gatt_events_task(&server, &conn, &nus);
                    let b = nus.run(server.deref(), &conn);
                    let c = select(a, b);
                    select(c, loopback_task(&nus)).await;
                }
                Err(e) => {
                    let e = defmt::Debug2Format(&e);
                    panic!("[adv] error: {:?}", e);
                }
            }
        }
    })
    .await;
}

/// This is a background task that is required to run forever alongside any other BLE tasks.
///
/// ## Alternative
///
/// If you didn't require this to be generic for your application, you could statically spawn this with i.e.
///
/// ```rust,ignore
///
/// #[embassy_executor::task]
/// async fn ble_task(mut runner: Runner<'static, SoftdeviceController<'static>>) {
///     runner.run().await;
/// }
///
/// spawner.must_spawn(ble_task(runner));
/// ```
async fn ble_task<C: Controller>(mut runner: Runner<'_, C>) {
    loop {
        if let Err(e) = runner.run().await {
            let e = defmt::Debug2Format(&e);
            panic!("[ble_task] error: {:?}", e);
        }
    }
}

/// Stream Events until the connection closes.
///
/// This function will handle the GATT events and process them.
/// This is how we interact with read and write requests.
async fn gatt_events_task(server: &Server<'_>, conn: &Connection<'_>, nus: &NusManager<'_, CriticalSectionRawMutex, 20, 200, 200> ) -> Result<(), trouble_host::Error> {
    loop {
        match conn.next().await {
            ConnectionEvent::Disconnected { reason } => {
                info!("[gatt] disconnected: {:?}", reason);
                break;
            }
            ConnectionEvent::Gatt { data } => {
                match data.process(server).await {
                    Ok(Some(event)) => {
                        let _ = nus.handle_gatt_event(&event, server.deref()).await;

                        // This step is also performed at drop(), but writing it explicitly is necessary
                        // in order to ensure reply is sent.
                        match event.accept() {
                            Ok(reply) => {
                                reply.send().await;
                            }
                            Err(e) => {
                                warn!("[gatt] error sending response: {:?}", e);
                            }
                        }
                    }
                    Ok(_) => {}
                    Err(e) => {
                        warn!("[gatt] error processing event: {:?}", e);
                    }
                }
            }
        }
    }
    info!("[gatt] task finished");
    Ok(())
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
async fn advertise<'a, C: Controller>(
    name: &'a str,
    peripheral: &mut Peripheral<'a, C>,
) -> Result<Connection<'a>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[Uuid::Uuid16([0x0f, 0x18])]),
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;
    let advertiser = peripheral
        .advertise(
            &Default::default(),
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..],
                scan_data: &[],
            },
        )
        .await?;
    info!("[adv] advertising");
    let conn = advertiser.accept().await?;
    info!("[adv] connection established");
    Ok(conn)
}