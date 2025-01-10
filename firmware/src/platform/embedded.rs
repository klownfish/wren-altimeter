#![allow(dead_code)]

use core::convert::Infallible;

use bmp388::{Bmp388, Bmp388Error};
use dummy_pin::DummyPin;
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_embedded_hal::shared_bus::SpiDeviceError;
use embassy_nrf::gpio::Output;
use embassy_nrf::peripherals::SPI2;
use embassy_nrf::spim;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use lis2dh12::{Lis2dh12, Lis2dh12Error};
use w25q32jv::{Error as W25q32jvError, W25q32jv};

use super::{Accelerometer, Barometer, BarometerData, FlashMemory};

pub type MySpiDevice = SpiDevice<'static, ThreadModeRawMutex, spim::Spim<'static, SPI2>, Output<'static>>;
pub type MySpiError = SpiDeviceError<embassy_nrf::spim::Error, Infallible>;

pub type BarometerType = Bmp388<MySpiDevice>;
pub type BarometerError = Bmp388Error<MySpiError>;
pub type AccelerometerType = Lis2dh12<MySpiDevice>;
pub type AccelerometerError = Lis2dh12Error<MySpiError>;

pub type FlashMemoryType = W25q32jv<MySpiDevice, DummyPin, DummyPin>;
pub type FlashMemoryError = W25q32jvError<MySpiError, Infallible>;

impl Barometer<BarometerError> for BarometerType {
    async fn read_temp_and_pressure(&mut self) -> Result<BarometerData, BarometerError> {
        let val = self.sensor_values().await?;
        Ok(BarometerData {
            pressure: val.pressure / 100.0,
            temperature: val.temperature,
        })
    }
}

impl Accelerometer<AccelerometerError> for AccelerometerType {
    async fn read_acceleration(&mut self) -> Result<[f32; 3], AccelerometerError> {
        let res = self.accel_norm().await?;
        let g = 9.82;
        Ok([res[0] * g, res[1] * g, res[2] * g])
    }
}

impl FlashMemory<FlashMemoryError> for FlashMemoryType {
    async fn read(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), FlashMemoryError> {
        self.read_async(address, buffer).await?;
        Ok(())
    }

    async fn write(&mut self, address: u32, data: &[u8]) -> Result<(), FlashMemoryError> {
        self.write_async(address, data).await?;
        Ok(())
    }

    async fn erase_all(&mut self) -> Result<(), FlashMemoryError> {
        self.erase_chip_async().await?;
        Ok(())
    }

    fn get_capacity() -> u32 {
        return FlashMemoryType::capacity() as u32;
    }
}
