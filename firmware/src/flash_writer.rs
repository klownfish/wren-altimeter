#[allow(unused_imports)]
#[cfg(target_os = "none")]
use defmt::{debug, error, info, warn};
use embassy_time::Instant;
#[allow(unused_imports)]
#[cfg(not(target_os = "none"))]
use log::{debug, error, info, warn};

use super::flight_sm::FlightState;
use super::platform::{FlashMemory, FlashMemoryType};

pub struct FlashWriter {
    index: u32,
    flash: FlashMemoryType,
}

#[repr(u8)]
enum MessageIds {
    Metadata = 1,
    Telemetry = 2,
    State = 3,
    PowerOn = 4,
    Debug = 5,
}

impl FlashWriter {
    pub async fn new(mut flash: FlashMemoryType) -> Result<Self, ()> {
        let mut max = FlashMemoryType::get_capacity();
        let mut min = 0;
        let mut buf = [0_u8; 10];

        while min < max {
            let mid = (min + max) / 2;
            let to_read = (FlashMemoryType::get_capacity() - mid).min(buf.len() as u32) as usize;
            let slice = &mut buf[0..to_read];

            flash.read(mid, slice).await.unwrap();

            if slice.iter().all(|&c| c == 0xff) {
                max = mid;
            } else {
                min = mid + 1;
            }
        }

        let out = Self {
            index: min,
            flash: flash,
        };
        Ok(out)
    }

    #[allow(unused)]
    pub async fn erase(&mut self) {
        self.flash.erase_all().await.unwrap();
        self.index = 0;
    }

    pub fn get_index(&self) -> u32 {
        self.index
    }

    pub fn get_size(&self) -> u32 {
        FlashMemoryType::get_capacity()
    }

    #[allow(unused)]
    pub async fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), ()> {
        self.flash.read(address, buf).await.unwrap();
        Ok(())
    }

    pub async fn write(&mut self, buf: &[u8]) -> Result<(), ()> {
        if self.index + buf.len() as u32 > self.get_size() {
            return Ok(());
        }
        self.flash.write(self.index, buf).await.unwrap();
        self.index += buf.len() as u32;
        Ok(())
    }

    pub async fn write_telem(&mut self, altitude: f32, velocity: f32, acceleration: f32) {
        let alt_int: u16 = (altitude * 4.0) as u16;
        let vel_int: u16 = ((velocity + 100.0) * 100.0) as u16;
        let accel_int: u16 = ((acceleration + 200.0) * 200.0) as u16;

        let alt_bytes = alt_int.to_le_bytes();
        let vel_bytes = vel_int.to_le_bytes();
        let accel_bytes = accel_int.to_le_bytes();

        let mut buf = [0_u8; 7];
        buf[0] = MessageIds::Telemetry as u8;
        buf[1..3].copy_from_slice(&alt_bytes);
        buf[3..5].copy_from_slice(&vel_bytes);
        buf[5..7].copy_from_slice(&accel_bytes);

        self.write(&buf).await.unwrap();
    }

    pub async fn write_metadata(&mut self, time: Instant, volt: f32) {
        let time_int: u32 = time.as_millis() as u32;
        let volt_int: u16 = (volt * 1000.0) as u16;

        let time_bytes = time_int.to_le_bytes();
        let volt_bytes = volt_int.to_le_bytes();
        let mut buf = [0_u8; 7];
        buf[0] = MessageIds::Metadata as u8;
        buf[1..5].copy_from_slice(&time_bytes);
        buf[5..7].copy_from_slice(&volt_bytes);
        self.write(&buf).await.unwrap();
    }

    pub async fn write_state(&mut self, state: FlightState) {
        let buf = [MessageIds::State as u8, state as u8];
        self.write(&buf).await.unwrap();
    }

    pub async fn write_power_on(&mut self) {
        let buf = [MessageIds::PowerOn as u8];
        self.write(&buf).await.unwrap();
    }

    #[allow(unused)]
    pub async fn write_debug(&mut self, acc_x: f32, acc_y: f32, acc_z: f32, pressure: f64, time: Instant) {
        let mut buf = [0_u8; 25];
        buf[0] = MessageIds::Debug as u8;
        buf[1..5].copy_from_slice(&(time.as_millis() as u32).to_ne_bytes());
        buf[5..9].copy_from_slice(&acc_x.to_ne_bytes());
        buf[9..13].copy_from_slice(&acc_y.to_ne_bytes());
        buf[13..17].copy_from_slice(&acc_z.to_ne_bytes());
        buf[17..25].copy_from_slice(&pressure.to_ne_bytes());
        self.write(&buf).await.unwrap();
    }
}
