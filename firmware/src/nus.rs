use embassy_sync::blocking_mutex::raw::RawMutex;
use trouble_host::prelude::{appearance, characteristic, descriptors, gatt_server, gatt_service, service, AdStructure, Address, Advertisement, AttributeServer, BleHostError, Characteristic, Connection, ConnectionEvent, Controller, DynamicAttributeServer, GapConfig, GattEvent, Host, HostResources, Peripheral, PeripheralConfig, Runner, Stack, Uuid, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE};
use embassy_sync::channel::{self, Channel};
#[allow(unused)]
use defmt::{debug, info, warn, error};

use heapless::Vec;

pub trait GattService<'server> {
    async fn run<M: RawMutex, const MAX: usize>(&self, server: &AttributeServer<'_, M, MAX>, conn: &Connection<'_>) -> Result<(), trouble_host::Error>;
    async fn handle_gatt_event<T: DynamicAttributeServer>(&self, event: &GattEvent<'_, 'server>, server: &'server T)  -> Result<(), trouble_host::Error>;

}



#[gatt_service(uuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")]
pub struct NusService {
    #[characteristic(uuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E", write)]
    pub rx: Vec<u8, 20>,

    #[characteristic(uuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E", notify)]
    pub tx: Vec<u8, 20>,
}


pub struct NusManager<'a, M: RawMutex, const MSG_SIZE: usize, const RX_SIZE: usize, const TX_SIZE: usize> {
    rx_channel: Channel<M, u8, RX_SIZE>,
    tx_channel: Channel<M, u8, TX_SIZE>,
    rx_characteristic: &'a Characteristic<Vec<u8, MSG_SIZE>>,
    tx_characteristic: &'a Characteristic<Vec<u8, MSG_SIZE>>,
}

impl<'server, M: RawMutex, const MSG_SIZE: usize, const RX_SIZE: usize, const TX_SIZE: usize> NusManager<'server, M, MSG_SIZE, RX_SIZE, TX_SIZE> {
    pub fn new(rx_characteristic: &'server Characteristic<Vec<u8, MSG_SIZE>>, tx_characteristic: &'server Characteristic<Vec<u8, MSG_SIZE>>) -> Self {
        Self {
            rx_characteristic: rx_characteristic,
            tx_characteristic: tx_characteristic,
            rx_channel: Channel::new(),
            tx_channel: Channel::new(),
        }
    }

    pub fn message_available(&self) -> bool {
        !self.rx_channel.is_empty()
    }

    pub async fn send_message(&self, buf: &[u8]) {
        let len = (buf.len() as u16).to_ne_bytes();
        self.tx_channel.send(len[0]).await;
        self.tx_channel.send(len[1]).await;
        for i in 0..buf.len() {
            self.tx_channel.send(buf[i]).await;
        }
    }

    pub fn tx_free_capacity(&self) -> usize {
        self.tx_channel.free_capacity()
    }

    pub async fn receive_message(&self, buf: &mut [u8]) -> usize{
        // Wait for length
        let mut length = [0_u8; 2];
        length[0] = self.rx_channel.receive().await;
        length[1] = self.rx_channel.receive().await;
        let length = u16::from_ne_bytes(length);
        info!("{}", length);
        // Wait for the buffer
        for i in 0..length as usize {
            buf[i] = self.rx_channel.receive().await;
        }
        length as usize
    }
}

impl<'server, M: RawMutex, const MSG_SIZE: usize, const RX_SIZE: usize, const TX_SIZE: usize> GattService<'server> for NusManager<'server, M, MSG_SIZE, RX_SIZE, TX_SIZE> {
    async fn handle_gatt_event<T: DynamicAttributeServer>(&self, event: &GattEvent<'_, 'server>, server: &'server T)  -> Result<(), trouble_host::Error> {
        _ = server;
        match &event {
            GattEvent::Read(event) => {
                if event.handle() == self.rx_characteristic.handle {
                    warn!("[NUS] Client tried reading the RX characteristic, this shouldn't happen");
                } else
                if event.handle() == self.tx_characteristic.handle {
                    warn!("[NUS] Client tried reading the TX characteristic, this shouldn't happen");
                }
            }
            GattEvent::Write(event) => {
                if event.handle() == self.rx_characteristic.handle {
                    info!("[NUS] Client wrote to the RX characteristic");

                    let length = event.value(self.rx_characteristic).unwrap().len().to_ne_bytes();
                    self.rx_channel.send(length[0]).await;
                    self.rx_channel.send(length[1]).await;

                    // Wait for the buffer
                    for v in event.value(&self.rx_characteristic).unwrap() {
                        self.rx_channel.send(v).await;
                    }

                } else
                if event.handle() == self.tx_characteristic.handle {
                    info!("[NUS] Client tried writing to the TX characteristic, this shouldn't happen");
                }
            }
        }
        Ok(())
    }

    async fn run<M2: RawMutex, const MAX: usize>(&self, server: &AttributeServer<'_, M2, MAX>, conn: &Connection<'_>) -> Result<(), trouble_host::Error> {
        loop {
            let mut vec: Vec<u8, MSG_SIZE> = Vec::new();

            // Wait for length
            let mut length = [0_u8; 2];
            length[0] = self.tx_channel.receive().await;
            length[1] = self.tx_channel.receive().await;
            let length = u16::from_ne_bytes(length);

            // Wait for the buffer
            for _ in 0..length as usize {
                vec.push(self.tx_channel.receive().await).unwrap();
            }
            self.tx_characteristic.notify(&server, &conn, &vec).await?;
        }
    }
}


/// Battery service
#[gatt_service(uuid = service::BATTERY)]
struct BatteryService {
    /// Battery Level
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, read, value = "Battery Level")]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 10)]
    level: u8,
}

struct BatteryManager<'a> {
    battery_characteristic: &'a Characteristic<u8>
}

impl<'server> BatteryManager<'server> {
    pub async fn update_level(&self, level: u8) {
        // self.battery_characteristic.set(server, value)
    }
}