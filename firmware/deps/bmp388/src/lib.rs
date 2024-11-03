//! A platform agnostic driver to interface with the BMP388 (pressure sensor)
//!
//! This driver is built using [`embedded-hal`](docs.rs/embedded-hal) traits.
//!
//! # Examples
//!
//! ```ignore
//! use bmp388::{BMP388, Addr, SensorData, PowerControl, PowerMode};
//! // depends on your board/chip
//! let i2c = todo!("Create the I2C interface");
//! // depends on your board/chip and/or crates you use.
//! let delay = todo!("Create a delay that implements `embedded_hal::delay::DelayMs`");
//! let mut sensor = BMP388::new(i2c, Addr::Primary as u8, delay).unwrap();
//!
//! // default `PowerMode` is `Sleep` so the first thing we need to change is the `PowerControl`.
//! // For continues reading using `Normal` mode,
//! // otherwise you can use `Force` but you have to set it before each sensor data reading.
//!
//! sensor.set_power_control(PowerControl::normal()).unwrap();
//!
//! let sensor_data: SensorData = sensor.sensor_values().unwrap();
//! ```

// #![deny(missing_docs)]
// #![deny(warnings)]
#![no_std]

use defmt::{info, error};

use config::{InterruptConfig, OversamplingConfig, Register};

pub mod config;

/// The expected value of the ChipId register
pub const CHIP_ID: u8 = 0x50;

pub struct Bmp388<Bus> {
    bus: Bus,
    // Temperature compensation
    temperature_calibration: TemperatureCalibration,
    // Pressure calibration
    pressure_calibration: PressureCalibration,
}

#[derive(Debug, Clone, Copy)]
pub enum Bmp388Error<BusError> {
    BusError(BusError),
    InvalidId,
}

impl<BusError> From<BusError> for Bmp388Error<BusError> {
    fn from(err: BusError) -> Self {
        Bmp388Error::BusError(err)
    }
}

/// Temperature calibration
#[derive(Debug, Default, Clone, Copy)]
struct TemperatureCalibration {
    dig_t1: u16,
    dig_t2: u16,
    dig_t3: i8,
}

impl TemperatureCalibration {
    /// Updates the clibration data for the Temperature
    pub fn update_calibration(&mut self, reg_data: [u8; 21]) {
        self.dig_t1 = (reg_data[0] as u16) | ((reg_data[1] as u16) << 8);
        self.dig_t2 = (reg_data[2] as u16) | ((reg_data[3] as u16) << 8);
        self.dig_t3 = reg_data[4] as i8;
    }
}

/// Pressure calibration
#[derive(Debug, Default, Clone, Copy)]
pub struct PressureCalibration {
    dig_p1: i16,
    dig_p2: i16,
    dig_p3: i8,
    dig_p4: i8,
    dig_p5: u16,
    dig_p6: u16,
    dig_p7: i8,
    dig_p8: i8,
    dig_p9: i16,
    dig_p10: i8,
    dig_p11: i8,
}

impl PressureCalibration {
    /// Updates the clibration data for the Pressure
    pub fn update_calibration(&mut self, reg_data: [u8; 21]) {
        self.dig_p1 = (reg_data[5] as i16) | ((reg_data[6] as i16) << 8);
        self.dig_p2 = (reg_data[7] as i16) | ((reg_data[8] as i16) << 8);
        self.dig_p3 = reg_data[9] as i8;
        self.dig_p4 = reg_data[10] as i8;
        self.dig_p5 = (reg_data[11] as u16) | ((reg_data[12] as u16) << 8);
        self.dig_p6 = (reg_data[13] as u16) | ((reg_data[14] as u16) << 8);
        self.dig_p7 = reg_data[15] as i8;
        self.dig_p8 = reg_data[16] as i8;
        self.dig_p9 = (reg_data[17] as i16) | ((reg_data[18] as i16) << 8);
        self.dig_p10 = reg_data[19] as i8;
        self.dig_p11 = reg_data[20] as i8;
    }
}

pub struct ErrorRegister {
    pub cmd: bool,
    pub fatal: bool,
    pub config: bool,
}

impl<Bus> Bmp388<Bus>  {
    /// Compensates a pressure value
    pub(crate) fn compensate_pressure(&self, uncompensated: u32, compensated_temp: f64) -> f64 {
        let uncompensated = uncompensated as f64;
        let p1 = ((self.pressure_calibration.dig_p1 as f64) - 16_384.0) / 1_048_576.0; //2^14 / 2^20
        let p2 = ((self.pressure_calibration.dig_p2 as f64) - 16_384.0) / 536_870_912.0; //2^14 / 2^29
        let p3 = (self.pressure_calibration.dig_p3 as f64) / 4_294_967_296.0; //2^32
        let p4 = (self.pressure_calibration.dig_p4 as f64) / 137_438_953_472.0; //2^37
        let p5 = (self.pressure_calibration.dig_p5 as f64) / 0.125; //2^-3
        let p6 = (self.pressure_calibration.dig_p6 as f64) / 64.0; //2^6
        let p7 = (self.pressure_calibration.dig_p7 as f64) / 256.0; //2^8
        let p8 = (self.pressure_calibration.dig_p8 as f64) / 32_768.0; //2^15
        let p9 = (self.pressure_calibration.dig_p9 as f64) / 281_474_976_710_656.0; //2^48
        let p10 = (self.pressure_calibration.dig_p10 as f64) / 281_474_976_710_656.0; //2^48
        let p11 = (self.pressure_calibration.dig_p11 as f64) / 36_893_488_147_419_103_232.0; //2^65

        let mut partial_data1 = p6 * compensated_temp;
        let mut partial_data2 = p7 * (compensated_temp * compensated_temp);
        let mut partial_data3 = p8 * (compensated_temp * compensated_temp * compensated_temp);
        let partial_out1 = p5 + partial_data1 + partial_data2 + partial_data3;

        partial_data1 = p2 * compensated_temp;
        partial_data2 = p3 * (compensated_temp * compensated_temp);
        partial_data3 = p4 * (compensated_temp * compensated_temp * compensated_temp);
        let partial_out2 = uncompensated * (p1 + partial_data1 + partial_data2 + partial_data3);

        partial_data1 = uncompensated * uncompensated;
        partial_data2 = p9 + p10 * compensated_temp;
        partial_data3 = partial_data1 * partial_data2;
        let partial_data4 = partial_data3 + (uncompensated * uncompensated * uncompensated) * p11;

        partial_out1 + partial_out2 + partial_data4
    }

    /// Compensates a temperature value
    pub(crate) fn compensate_temp(&self, uncompensated: u32) -> f64 {
        let t1 = (self.temperature_calibration.dig_t1 as f64) / 0.00390625; //2^-8
        let t2 = (self.temperature_calibration.dig_t2 as f64) / 1_073_741_824.0; //2^30
        let t3 = (self.temperature_calibration.dig_t3 as f64) / 281_474_976_710_656.0; //2^48

        let partial_data1 = (uncompensated as f64) - t1;
        let partial_data2 = partial_data1 * t2;

        partial_data2 + (partial_data1 * partial_data1) * t3
    }
}

impl<Spi: embedded_hal_async::spi::SpiDevice> Bmp388<Spi>  {
    /// Creates new BMP388 driver
    ///
    /// The Delay is used to correctly wait for the calibration data after resetting the chip.
    pub async fn new(
        spi: Spi,
        delay: &mut impl embedded_hal_async::delay::DelayNs,
    ) -> Result<Bmp388<Spi>, Bmp388Error<Spi::Error>>
    {
        let mut chip = Bmp388 {
            bus: spi,
            temperature_calibration: TemperatureCalibration::default(),
            pressure_calibration: PressureCalibration::default(),
        };
        let id = chip.id().await?;
        if id == CHIP_ID {
            chip.reset().await?;
            // without this the first few bytes of calib data could be incorrectly zero
            delay.delay_ms(10).await;
            chip.read_calibration().await?;
            info!("Bmp388 initialized");
            Ok(chip)

        } else {
            error!("Bmp388 failed to initialize, id: {}", id);

            Err(Bmp388Error::InvalidId)
        }
    }

    pub(crate) async fn read_calibration(&mut self) -> Result<(), Bmp388Error<Spi::Error>> {
        let mut data: [u8; 23] = [0; 23];
        data[0] = Register::calib00 as u8;
        self.read_bytes(&mut data).await?;

        let mut new_data: [u8; 21] = [0; 21]; // bruh moment
        new_data.copy_from_slice(&data[2..]);

        self.temperature_calibration.update_calibration(new_data);
        self.pressure_calibration.update_calibration(new_data);

        Ok(())
    }

    /// Reads and returns sensor values
    pub async fn sensor_values(&mut self) -> Result<SensorData, Bmp388Error<Spi::Error>> {
        let mut data: [u8; 8] = [Register::sensor_data as u8, 0, 0, 0, 0, 0, 0, 0];
        self.read_bytes(&mut data).await?;

        let uncompensated_press = (data[2] as u32) | (data[3] as u32) << 8 | (data[4] as u32) << 16;
        let uncompensated_temp = (data[5] as u32) | (data[6] as u32) << 8 | (data[7] as u32) << 16;

        let temperature = self.compensate_temp(uncompensated_temp);
        let pressure = self.compensate_pressure(uncompensated_press, temperature);

        Ok(SensorData {
            pressure,
            temperature,
        })
    }

    /// Sets power settings
    pub async fn set_power_control(&mut self, new: PowerControl) -> Result<(), Bmp388Error<Spi::Error>> {
        self.write_register(Register::pwr_ctrl, new.to_reg()).await?;
        Ok(())
    }

    /// Gets power settings
    pub async fn power_control(&mut self) -> Result<PowerControl, Bmp388Error<Spi::Error>> {
        let value = self.read_register(Register::pwr_ctrl).await?;

        Ok(PowerControl::from_reg(value))
    }

    /// Sets sampling rate
    pub async fn set_sampling_rate(&mut self, new: SamplingRate) -> Result<(), Bmp388Error<Spi::Error>> {
        let sampling_rate = new as u8;
        self.write_register(Register::odr, sampling_rate).await?;
        Ok(())
    }

    /// Returns current sampling rate
    pub async fn sampling_rate(&mut self) -> Result<SamplingRate, Bmp388Error<Spi::Error>> {
        let value = self.read_register(Register::odr).await?;
        let sampling_rate = SamplingRate::from_reg(value);

        Ok(sampling_rate)
    }

    /// Returns current filter
    pub async fn filter(&mut self) -> Result<Filter, Bmp388Error<Spi::Error>> {
        let config = self.read_register(Register::config).await?;
        let filter = Filter::from_reg(config);

        Ok(filter)
    }

    /// Sets filter
    pub async fn set_filter(&mut self, new: Filter) -> Result<(), Bmp388Error<Spi::Error>> {
        self.write_register(Register::config, new.to_reg()).await?;
        Ok(())
    }

    /// Sets oversampling configuration
    pub async fn set_oversampling(&mut self, new: OversamplingConfig) -> Result<(), Bmp388Error<Spi::Error>> {
        self.write_register(Register::osr, new.to_reg()).await?;
        Ok(())
    }

    /// Get oversampling configuration
    pub async fn oversampling(&mut self) -> Result<OversamplingConfig, Bmp388Error<Spi::Error>> {
        let value = self.read_register(Register::osr).await?;
        Ok(OversamplingConfig::from_reg(value))
    }

    /// Sets interrupt configuration
    pub async fn set_interrupt_config(&mut self, new: InterruptConfig) -> Result<(), Bmp388Error<Spi::Error>> {
        self.write_register(Register::int_ctrl, new.to_reg()).await?;
        Ok(())
    }

    /// Returns current interrupt configuration
    pub async fn interrupt_config(&mut self) -> Result<InterruptConfig, Bmp388Error<Spi::Error>> {
        let value = self.read_register(Register::int_ctrl).await?;

        Ok(InterruptConfig::from_reg(value))
    }

    /// Get the status register
    pub async fn status(&mut self) -> Result<Status, Bmp388Error<Spi::Error>> {
        let status = self.read_register(Register::status).await?;

        Ok(Status::from_reg(status))
    }

    /// Get the error register
    pub async fn error(&mut self) -> Result<ErrorRegister, Bmp388Error<Spi::Error>> {
        let error = self.read_register(Register::err).await?;
        Ok(ErrorRegister {
            fatal: error & (1 << 0) != 0,
            cmd: error & (1 << 1) != 0,
            config: error & (1 << 2) != 0,
        })
    }

    /// Returns device id
    pub async fn id(&mut self) -> Result<u8, Bmp388Error<Spi::Error>> {
        let id = self.read_register(Register::id).await?;
        Ok(id)
    }

    /// Software reset, emulates POR
    pub async fn reset(&mut self) -> Result<(), Bmp388Error<Spi::Error>> {
        self.write_register(Register::cmd, 0xB6).await?; // Magic from documentation
        Ok(())
    }

    async fn write_register(&mut self, reg: Register, byte: u8) -> Result<(), Spi::Error> {
        let buf: [u8; 2] = [reg as u8, byte];
        self.write_bytes(&buf).await
    }

    async fn read_register(&mut self, reg: Register) -> Result<u8, Spi::Error> {
        let mut buf: [u8; 3] = [reg as u8 | (1 << 7), 0, 0];
        self.bus.transfer_in_place(&mut buf[0..3]).await?;
        Ok(buf[2])
    }

    async fn read_bytes(&mut self, data: &mut [u8]) -> Result<(), Spi::Error> {
        data[0] |= 1 << 7; // set bit 7 to read

        self.bus.transfer_in_place(data).await?;
        Ok(())
    }

    async fn write_bytes(&mut self, data: &[u8]) -> Result<(), Spi::Error> {
        self.bus.write(data).await
    }
}

/// Status
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct Status {
    ///Indicates whether chip is ready for a command
    pub command_ready: bool,
    ///Indicates whether pressure data is ready
    pub pressure_data_ready: bool,
    ///Indicates whether temperature data is ready
    pub temperature_data_ready: bool,
}

impl Status {
    /// Creates the Status from a register value
    pub(crate) fn from_reg(value: u8) -> Self {
        Self {
            command_ready: value & (1 << 4) != 0,
            pressure_data_ready: value & (1 << 5) != 0,
            temperature_data_ready: value & (1 << 6) != 0,
        }
    }
}

#[derive(Debug, Copy, Clone)]
/// Sensor data
pub struct SensorData {
    ///The measured pressure: Pascals (Pa)
    pub pressure: f64,
    /// The measured temperature: Degrees celsius (°C)
    pub temperature: f64,
}

/// Power Control
///
/// Register: `PWR_CTRL`
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct PowerControl {
    /// Pressure sensor enable
    pub pressure_enable: bool,
    /// Temperature sensor enable
    pub temperature_enable: bool,
    /// Power mode
    ///
    /// On `PowerMode::Forced`, you need to set the `PowerMode`
    /// after each sensor values reading.
    pub mode: PowerMode,
}

impl PowerControl {
    /// Create a new PowerControl with `PowerMode::Normal` and enable both sensors.
    pub fn normal() -> Self {
        Self {
            pressure_enable: true,
            temperature_enable: true,
            mode: PowerMode::Normal,
        }
    }

    pub fn from_reg(reg: u8) -> Self {
        let pressure_enable = (reg & 0b1) != 0;
        let temperature_enable = (reg & 0b10) != 0;
        let mode = match reg & (0b11 << 4) >> 4 {
            x if x == PowerMode::Forced as u8 => PowerMode::Forced,
            x if x == PowerMode::Normal as u8 => PowerMode::Normal,
            // in any other case, we assume Sleep as it's the default
            _ => PowerMode::Sleep,
        };

        Self {
            pressure_enable,
            temperature_enable,
            mode,
        }
    }

    pub fn to_reg(self) -> u8 {
        let mode = (self.mode as u8) << 4;
        let temp_en = (self.temperature_enable as u8) << 1;
        let press_en = self.pressure_enable as u8;

        mode | temp_en | press_en
    }
}

impl Default for PowerControl {
    /// default value for the register is `0x00`
    fn default() -> Self {
        Self {
            pressure_enable: false,
            temperature_enable: false,
            mode: PowerMode::default(),
        }
    }
}

/// Output mode for interrupt pin
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum OutputMode {
    ///Push-pull output mode
    PushPull = 0,
    ///Open-drain output mode
    OpenDrain = 1,
}

/// Standby time in ms (ODR reg)
///
/// ODR register (0x1D) = 0x00 - values from 0-17 - Subsampling
///
/// Default value: 0x00
#[derive(Default, Debug, Copy, Clone, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum SamplingRate {
    /// Prescaler 1 (5ms, 200 Hz)
    ///
    /// Description: ODR 200 Hz
    #[default]
    ms5 = 0x00,
    /// Prescaler 2 (10ms, 100 Hz)
    ///
    /// Description: ODR 100 Hz
    ms10 = 0x01,
    /// Prescaler 4 (20ms, 50 Hz)
    ///
    /// Description: ODR 50 Hz
    ms20 = 0x02,
    /// Prescaler 8 (40ms, 25 Hz)
    ///
    /// Description: ODR 25 Hz
    ms40 = 0x03,
    /// Prescaler 16 (80ms, 25/2 Hz)
    ///
    /// Description: ODR 25/2 Hz
    ms80 = 0x04,
    /// Prescaler 32 (160ms, 25/4 Hz)
    ///
    /// Description: ODR 25/4 Hz
    ms160 = 0x05,
    /// Prescaler 64 (320ms, 25/8 Hz)
    ///
    /// Description: ODR 25/8 Hz
    ms320 = 0x06,
    /// Prescaler 128 (640ms, 25/18 Hz)
    ///
    /// Description: ODR 25/18 Hz
    ms640 = 0x07,
    /// Prescaler 256 (1.280s, 25/32 Hz)
    ///
    /// Description: ODR 25/32 Hz
    ms1_280 = 0x08,
    /// Prescaler 512 (2.560s, 25/64 Hz)
    ///
    /// Description: ODR 25/64 Hz
    ms2_560 = 0x09,
    /// Prescaler 1024 (5.120s, 25/128 Hz)
    ///
    /// Description: ODR 25/128 Hz
    ms5_120 = 0x0A,
    /// Prescaler 2048 (10.24s, 25/256 Hz)
    ///
    /// Description: ODR 25/256 Hz
    ms1_024 = 0x0B,
    /// Prescaler 4096 (20.48s, 25/512 Hz)
    ///
    /// Description: ODR 25/512 Hz
    ms2_048 = 0x0C,
    /// Prescaler 8192 (40.96s, 25/1024 Hz)
    ///
    /// Description: ODR 25/1024 Hz
    ms4_096 = 0x0D,
    /// Prescaler 16384 (81.92s, 25/2048 Hz)
    ///
    /// Description: ODR 25/2048 Hz
    ms8_192 = 0x0E,
    /// Prescaler 32768 (163.84s, 25/4096 Hz)
    ///
    /// Description: ODR 25/4096 Hz
    ms16_384 = 0x0F,
    /// Prescaler 65536 (327.68s, 25/8192 Hz)
    ///
    /// Description: ODR 25/8192 Hz
    ms32_768 = 0x10,
    /// Prescaler 131072 (655.36s, 25/16384 Hz)
    ///
    /// Description: ODR 25/16384 Hz
    ms65_536 = 0x11,
}

impl SamplingRate {
    /// For any unknown value it will use the default of `5 ms` or `200 Hz`.
    pub fn from_reg(value: u8) -> Self {
        match value {
            x if x == SamplingRate::ms5 as u8 => SamplingRate::ms5,
            x if x == SamplingRate::ms10 as u8 => SamplingRate::ms10,
            x if x == SamplingRate::ms20 as u8 => SamplingRate::ms20,
            x if x == SamplingRate::ms40 as u8 => SamplingRate::ms40,
            x if x == SamplingRate::ms80 as u8 => SamplingRate::ms80,
            x if x == SamplingRate::ms160 as u8 => SamplingRate::ms160,
            x if x == SamplingRate::ms320 as u8 => SamplingRate::ms320,
            x if x == SamplingRate::ms640 as u8 => SamplingRate::ms640,
            x if x == SamplingRate::ms1_280 as u8 => SamplingRate::ms1_280,
            x if x == SamplingRate::ms2_560 as u8 => SamplingRate::ms2_560,
            x if x == SamplingRate::ms5_120 as u8 => SamplingRate::ms5_120,
            x if x == SamplingRate::ms1_024 as u8 => SamplingRate::ms1_024,
            x if x == SamplingRate::ms2_048 as u8 => SamplingRate::ms2_048,
            x if x == SamplingRate::ms4_096 as u8 => SamplingRate::ms4_096,
            x if x == SamplingRate::ms8_192 as u8 => SamplingRate::ms8_192,
            x if x == SamplingRate::ms16_384 as u8 => SamplingRate::ms16_384,
            x if x == SamplingRate::ms32_768 as u8 => SamplingRate::ms32_768,
            x if x == SamplingRate::ms65_536 as u8 => SamplingRate::ms65_536,
            // for any other value, use the default of 5 ms
            _ => SamplingRate::default(),
        }
    }
}

/// The time constant of IIR filter
#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
#[allow(non_camel_case_types)]
// #[repr(u8)]
pub enum Filter {
    #[default]
    ///off
    c0 = 0b000,
    ///Coefficient 1
    c1 = 0b001,
    ///Coefficient 3
    c3 = 0b010,
    ///Coefficient 7
    c7 = 0b011,
    ///Coefficient 15
    c15 = 0b100,
    ///Coefficient 31
    c31 = 0b101,
    ///Coefficient 63
    c63 = 0b110,
    ///Coefficient 127
    c127 = 0b111,
}

impl Filter {
    /// Parses the filter value from a the Config register
    pub fn from_reg(config_reg: u8) -> Self {
        match config_reg << 1 {
            x if x == Filter::c0 as u8 => Filter::c0,
            x if x == Filter::c1 as u8 => Filter::c1,
            x if x == Filter::c3 as u8 => Filter::c3,
            x if x == Filter::c7 as u8 => Filter::c7,
            x if x == Filter::c15 as u8 => Filter::c15,
            x if x == Filter::c31 as u8 => Filter::c31,
            x if x == Filter::c63 as u8 => Filter::c63,
            x if x == Filter::c127 as u8 => Filter::c127,
            // for any other value use default of c0 (off)
            _ => Filter::c0,
        }
    }

    /// Creates a value for the Config register
    pub fn to_reg(&self) -> u8 {
        (*self as u8) << 1
    }
}

/// Oversampling
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum Oversampling {
    /// x1
    ///
    /// Pressure setting - Ultra low power
    /// Typical pressure resolution - 16 bit / 2.64 Pa
    /// Recommended temperature oversampling: x1
    ///
    /// Typical temperature resolution - 16 bit / 0.0050 °C
    x1 = 0b000,
    /// x2
    ///
    /// Pressure setting - Low power
    /// Typical pressure resolution - 17 bit / 1.32 Pa
    /// Recommended temperature oversampling: x1
    ///
    /// Typical temperature resolution - 17 bit / 0.0025 °C
    x2 = 0b001,
    /// x4
    ///
    /// Pressure setting - Standard resolution
    /// Typical pressure resolution - 18 bit / 0.66 Pa
    /// Recommended temperature oversampling: x1
    ///
    /// Typical temperature resolution - 18 bit / 0.0012 °C
    x4 = 0b010,
    /// x8
    ///
    /// Pressure setting - High resolution
    /// Typical pressure resolution - 19 bit / 0.33 Pa
    /// Recommended temperature oversampling: x1
    ///
    /// Typical temperature resolution - 19 bit / 0.0006 °C
    x8 = 0b011,
    /// x16
    ///
    /// Pressure setting - Ultra high resolution
    /// Typical pressure resolution - 20 bit / 0.17 Pa
    /// Recommended temperature oversampling: x2
    ///
    /// Typical temperature resolution - 20 bit / 0.0003 °C
    x16 = 0b100,
    /// x32
    ///
    /// Pressure setting - Highest resolution
    /// Typical pressure resolution - 21 bit / 0.085 Pa
    /// Recommended temperature oversampling: x2
    ///
    /// Typical temperature resolution - 21 bit / 0.00015 °C
    x32 = 0b101,
}

/// PowerMode
///
///
/// ```
/// use bmp388::PowerMode;
///
/// let default = PowerMode::default();
/// assert_eq!(PowerMode::Sleep, default);
/// ```
#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
#[repr(u8)]
pub enum PowerMode {
    /// Sleep
    ///
    /// Default Power model on start-up
    #[default]
    Sleep = 0b00,
    /// Forced
    ///
    /// In `Forced` mode, after each measurement you need to set the `PowerMode` to `Forced` again.
    Forced = 0b01,
    /// Normal
    Normal = 0b11,
}