use std::fs::File;
use std::path::Path;
use std::io::{self, BufRead};
use std::convert::Infallible;
use std::env;
use std::io::Write;

#[allow(unused_imports)]
use log::{info, warn, error, debug};

use embassy_time::Instant;

use bmp388::Bmp388Error;
use lis2dh12::Lis2dh12Error;
use w25q32jv::{Error as W25q32jvError};

use super::{Accelerometer, Barometer, BarometerData, FlashMemory};

pub type BarometerType = MockBarometer;
pub type BarometerError = Bmp388Error<Infallible>;
pub type AccelerometerType = MockAccelerometer;
pub type AccelerometerError = Lis2dh12Error<Infallible>;

pub type FlashMemoryType = MockFlashMemory;
pub type FlashMemoryError = W25q32jvError<Infallible, Infallible>;

pub struct MockBarometer {
    data: Vec<(f64, f64)>
}

pub struct MockAccelerometer {
    data: Vec<(f64, [f64; 3])>
}

pub struct MockFlashMemory {
    file: File
}

impl MockBarometer {
    pub fn new() -> Self {
        let source_file = file!(); // Gets the current file name (relative to the crate root)
        let source_dir = Path::new(source_file).parent().unwrap(); // Gets the directory of the source file
        let data_path = source_dir.join("data/skyward_baro.csv");
        let mut file = File::open(&data_path).unwrap();

        let reader = io::BufReader::new(file);

        let mut data = Vec::new();

        for (index, line) in reader.lines().enumerate() {
            let line = line.unwrap();
            if index == 0 {
                continue;
            }

            let mut parts = line.split(',');
            if let (Some(time), Some(pressure)) = (parts.next(), parts.next()) {
                let time: f64 = time.trim().parse().expect("Invalid time format");
                let pressure: f64 = pressure.trim().parse().expect("Invalid pressure format");
                data.push((time, pressure));
            } else {
                eprintln!("Skipping invalid line: {}", line);
            }
        }

        Self {
            data: data
        }
    }
}

impl MockAccelerometer {
    pub fn new() -> Self {
        let source_file = file!(); // Gets the current file name (relative to the crate root)
        let source_dir = Path::new(source_file).parent().unwrap(); // Gets the directory of the source file
        let data_path = source_dir.join("data/skyward_imu.csv");
        let input = std::fs::read_to_string(&data_path).unwrap();

        let mut data = Vec::new();

        for line in input.lines() {
            if line.starts_with("time") {
                continue;
            }

            let values: Vec<&str> = line.split(',').collect();

            let time: f64 = values[0].parse().unwrap();

            let acc_x: f64 = values[1].parse().unwrap();
            let acc_y: f64 = values[2].parse().unwrap();
            let acc_z: f64 = values[3].parse().unwrap();

            data.push((time, [acc_x, acc_y, acc_z]));
        }


        Self {
            data: data
        }
    }
}

impl MockFlashMemory {
    pub fn new() -> Self {
        let tmp_dir = env::temp_dir();
        let path = tmp_dir.join("wren_flash.bin");

        let mut file = match File::create(&path) {
            Err(why) => panic!("couldn't create flash file {}", why),
            Ok(file) => file,
        };

        Self {
            file: file
        }
    }
}

impl Barometer<BarometerError> for BarometerType {
    async fn read_temp_and_pressure(&mut self) -> Result<BarometerData, BarometerError> {
        let now = Instant::now().as_millis() as f64 / 1000.0 - 15.0;
        for (time, pressure) in &self.data {
            if *time > now {
                return Ok(BarometerData {
                    pressure: *pressure / 100.0,
                    temperature: 22.0,
                })
            }
        }

        Ok(BarometerData {
            pressure: self.data[self.data.len() - 1].1 / 100.0,
            temperature: 0.0,
        })
    }
}

impl Accelerometer<AccelerometerError> for AccelerometerType {
    async fn read_acceleration(&mut self) -> Result<[f32; 3], AccelerometerError> {
        let now = Instant::now().as_millis() as f64 / 1000.0 - 15.0;
        for (time, val) in &self.data {
            if *time > now {
                return Ok([
                    val[0] as f32,
                    val[1] as f32,
                    val[2] as f32,
                ])
            }
        }
        let val = self.data[self.data.len() - 1].1;
        Ok([
            val[0] as f32,
            val[1] as f32,
            val[2] as f32,
        ])
    }
}

impl FlashMemory<FlashMemoryError> for FlashMemoryType {
    async fn read(&mut self, address: u32, buffer: &mut [u8]) -> Result<(), FlashMemoryError> {
        buffer.fill(0xff);
        Ok(())
    }

    async fn write(&mut self, address: u32, data: &[u8]) -> Result<(), FlashMemoryError> {
        self.file.write_all(data).unwrap();
        Ok(())
    }

    async fn erase_all(&mut self) -> Result<(), FlashMemoryError> {
        Ok(())
    }

    fn get_capacity() -> u32 {
        return 50000 as u32;
    }
}
