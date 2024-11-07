use crate::{flight_sm::FlightState, W25q32jv};
use dummy_pin::DummyPin;
use embassy_time::Instant;

pub struct FlashStuff<T: embedded_hal_async::spi::SpiDevice> {
    index: u32,
    flash: W25q32jv<T, DummyPin, DummyPin>
}

#[repr(u8)]
enum MessageIds {
    Metadata = 1,
    Telemetry = 2,
    State = 3
}

impl<T: embedded_hal_async::spi::SpiDevice> FlashStuff<T> {
    pub async fn new(spi_device: T) -> Result<Self, ()> {
        let mut flash = W25q32jv::<T, DummyPin, DummyPin>::new(spi_device, DummyPin::new_low(), DummyPin::new_low()).unwrap();

        let mut max = w25q32jv::CAPACITY;
        let mut min = 0;
        let mut buf = [0_u8; 10];

        while min < max {
            let mid = (min + max) / 2;
            let to_read = (w25q32jv::CAPACITY - mid).min(buf.len() as u32) as usize;
            let slice = &mut buf[0..to_read];

            flash.read_async(mid, slice).await.unwrap();

            if slice.iter().all(|&c| c == 0xff) {
                max = mid;
            } else {
                min = mid + 1;
            }
        }

        let out = Self {
            index: min,
            flash: flash
        };
        Ok(out)
    }

    pub async fn erase(&mut self) {
        self.flash.erase_chip_async().await.unwrap();
    }

    pub fn get_index(&self) -> u32 {
        self.index
    }

    pub fn get_size(&self) -> u32 {
        w25q32jv::CAPACITY
    }

    pub async fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), ()> {
        self.flash.read_async(address, buf).await.unwrap();
        Ok(())
    }

    pub async fn write(&mut self, buf: &[u8]) -> Result<(), ()> {
        self.flash.write_async(self.index, buf).await.unwrap();
        self.index += buf.len() as u32;
        Ok(())
    }

    pub async fn write_telem(&mut self, altitude: f32, velocity: f32, acceleration: f32) {
        let alt_int: u16 = (altitude * 4.0) as u16;
        let vel_int: u16 = (velocity * 100.0) as u16;
        let accel_int: u16 = (acceleration * 200.0) as u16;

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
        let time_int: u16 = time.as_millis() as u16;
        let volt_int: u16 = (volt * 1000.0) as u16;

        let time_bytes = time_int.to_le_bytes();
        let volt_bytes = volt_int.to_le_bytes();
        let mut buf = [0_u8; 5];
        buf[0] = MessageIds::Metadata as u8;
        buf[1..3].copy_from_slice(&time_bytes);
        buf[3..5].copy_from_slice(&volt_bytes);
        self.write(&buf).await.unwrap();
    }

    pub async fn write_state(&mut self, state: FlightState) {
        let buf = [MessageIds::State as u8, state as u8];
        self.write(&buf).await.unwrap();
    }
}