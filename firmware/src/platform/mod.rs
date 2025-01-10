#![allow(dead_code)]

#[cfg(target_os = "none")]
mod embedded;
#[cfg(target_os = "none")]
pub use embedded::*;

#[cfg(not(target_os = "none"))]
mod native;
#[cfg(not(target_os = "none"))]
pub use native::*;

pub trait FlashMemory<E> {
    async fn read(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), E>;
    async fn write(&mut self, address: u32, data: &[u8]) -> Result<(), E>;
    async fn erase_all(&mut self) -> Result<(), E>;
    fn get_capacity() -> u32;
}

pub struct BarometerData {
    pub pressure: f64,
    pub temperature: f64,
}
pub trait Barometer<E> {
    async fn read_temp_and_pressure(&mut self) -> Result<BarometerData, E>;
}

pub trait Accelerometer<E> {
    async fn read_acceleration(&mut self) -> Result<[f32; 3], E>;
}
