//! Platform-agnostic LIS2DH12 accelerometer driver which uses I2C via
//! [embedded-hal] and implements the [`Accelerometer` trait][trait]
//! from the `accelerometer` crate.
//!
//! [embedded-hal]: https://docs.rs/embedded-hal
//! [trait]: https://docs.rs/accelerometer/latest/accelerometer/trait.Accelerometer.html
//!

#![deny(warnings)]
#![no_std]
#![forbid(unsafe_code)]

mod reg;

use core::fmt::Debug;
use core::marker::PhantomData;

use cast::u16;
use cast::{f32, i16};

use crate::reg::*;
pub use crate::reg::{Aoi6d, FifoMode, FullScale, Mode, Odr};

#[allow(unused)]
use defmt::{error, info};

/// Data status structure,
/// decoded from STATUS_REG register
#[derive(Debug)]
pub struct DataStatus {
    /// ZYXOR bit
    pub zyxor: bool,
    /// (XOR, YOR, ZOR) bits
    pub xyzor: (bool, bool, bool),
    /// ZYXDA bit
    pub zyxda: bool,
    /// (XDA, YDA, ZDA) bits
    pub xyzda: (bool, bool, bool),
}

/// `LIS2DH12` driver
pub struct Lis2dh12<Bus> {
    bus: Bus,
    /// Current full-scale
    fs: FullScale,
}

/// Errors returned from the [detect_i2c_addr]
#[derive(Debug, Clone, Copy)]
pub enum Lis2dh12Error<BusError> {
    /// Other I2C error trying to detect a device address.
    BusError(BusError),
    /// Invalid device ID read from device.
    InvalidDeviceId,
}

impl<BusError> From<BusError> for Lis2dh12Error<BusError> {
    fn from(err: BusError) -> Self {
        Lis2dh12Error::BusError(err)
    }
}

/// Interrupt setting and status
pub struct Int<'a, REG, Bus> {
    dev: &'a mut Lis2dh12<Bus>,
    reg: PhantomData<REG>,
}

impl<Spi: embedded_hal_async::spi::SpiDevice> Lis2dh12<Spi> {
    /// Create a new `LIS2DH12` driver from the given `I2C` peripheral
    pub async fn new(spi: Spi) -> Result<Self, Lis2dh12Error<Spi::Error>> {
        let mut dev = Self {
            bus: spi,
            fs: FullScale::G2,
        };

        // Ensure we have the correct device ID
        let id = dev.get_device_id().await?;
        if id != DEVICE_ID {
            error!("invalid device ID: {}", id);
            Err(Lis2dh12Error::InvalidDeviceId)
        } else{
            Ok(dev)
        }

    }

    pub async fn enabled_adcs(&mut self) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.write_reg(Register::TEMP_CFG_REG, ADC_EN).await?;
        Ok(())
    }

    /// `WHO_AM_I` register
    pub async fn get_device_id(&mut self) -> Result<u8, Lis2dh12Error<Spi::Error>> {
        let id = self.read_reg(Register::WHO_AM_I).await?;
        Ok(id)
    }

    /// Operating mode selection,
    /// `CTRL_REG1`: `LPen` bit,
    /// `CTRL_REG4`: `HR` bit
    pub async fn set_mode(&mut self, mode: Mode) -> Result<(), Lis2dh12Error<Spi::Error>> {
        match mode {
            Mode::LowPower => {
                self.reg_reset_bits(Register::CTRL_REG4, HR).await?;
                self.reg_set_bits(Register::CTRL_REG1, LPen).await?;
            }
            Mode::Normal => {
                self.reg_reset_bits(Register::CTRL_REG1, LPen).await?;
                self.reg_reset_bits(Register::CTRL_REG4, HR).await?;
            }
            Mode::HighResolution => {
                self.reg_reset_bits(Register::CTRL_REG1, LPen).await?;
                self.reg_set_bits(Register::CTRL_REG4, HR).await?;
            }
        }
        Ok(())
    }

    /// Data rate selection,
    /// `CTRL_REG1`: `ODR`
    pub async fn set_odr(&mut self, odr: Odr) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.modify_reg(Register::CTRL_REG1, |v| {
            (v & !ODR_MASK) | ((odr as u8) << 4)
        }).await?;
        // By design, when the device from high-resolution configuration (HR) is set to power-down mode (PD),
        // it is recommended to read register REFERENCE (26h) for a complete reset of the filtering block
        // before switching to normal/high-performance mode again.
        if let Odr::PowerDown = odr {
            self.get_ref().await?;
        }
        Ok(())
    }

    /// X,Y,Z-axis enable,
    /// `CTRL_REG1`: `Xen`, `Yen`, `Zen`
    pub async fn enable_axis(&mut self, (x, y, z): (bool, bool, bool)) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.modify_reg(Register::CTRL_REG1, |mut v| {
            v &= !(Xen | Yen | Zen); // disable all axises
            v |= if x { Xen } else { 0 };
            v |= if y { Yen } else { 0 };
            v |= if z { Zen } else { 0 };
            v
        }).await?;
        Ok(())
    }

    /// Enable high-pass filter for CLICK/IA2/IA1
    pub async fn enable_hp_filter(
        &mut self,
        click: bool,
        ia2: bool,
        ia1: bool,
    ) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.modify_reg(Register::CTRL_REG2, |mut v| {
            v &= !(HPCLICK | HP_IA2 | HP_IA1); // disable all filters
            v |= if click { HPCLICK } else { 0 };
            v |= if ia2 { HP_IA2 } else { 0 };
            v |= if ia1 { HP_IA1 } else { 0 };
            v
        }).await?;
        Ok(())
    }

    /// `CLICK` interrupt on `INT1` pin,
    /// `CTRL_REG3`: `I1_CLICK`
    pub async fn enable_i1_click(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG3, I1_CLICK, enable).await?;
        Ok(())
    }

    /// `IA1` interrupt on `INT1` pin,
    /// `CTRL_REG3`: `I1_IA1`
    pub async fn enable_i1_ia1(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG3, I1_IA1, enable).await?;
        Ok(())
    }

    /// `IA2` interrupt on `INT1` pin,
    /// `CTRL_REG3`: `I1_IA2`
    pub async fn enable_i1_ia2(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG3, I1_IA2, enable).await?;
        Ok(())
    }

    /// `ZYXDA` interrupt on `INT1` pin,
    /// `CTRL_REG3`: `I2_ZYXDA`
    pub async fn enable_i1_zyxda(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG3, I1_ZYXDA, enable).await?;
        Ok(())
    }

    /// FIFO watermark on `INT1` pin,
    /// `CTRL_REG3`: `I2_ZYXDA`
    pub async fn enable_i1_wtm(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG3, I1_WTM, enable).await?;
        Ok(())
    }

    /// Get the amount of samples currently stored in the FIFO queue
    pub async fn get_stored_samples(&mut self) -> Result<u8, Lis2dh12Error<Spi::Error>> {
        let value = self.read_reg(Register::FIFO_SRC_REG).await?;
        Ok(value & FSS)
    }

    /// FIFO overrun on `INT1` pin,
    /// `CTRL_REG3`: `I1_OVERRUN`
    pub async fn enable_i1_overrun(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG3, I1_OVERRUN, enable).await?;
        Ok(())
    }

    /// Block data update,
    /// `CTRL_REG4`: `BDU`
    pub async fn set_bdu(&mut self, bdu: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG4, BDU, bdu).await?;
        Ok(())
    }

    /// Full-scale selection,
    /// `CTRL_REG4`: `FS`
    pub async fn set_fs(&mut self, fs: FullScale) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.modify_reg(Register::CTRL_REG4, |v| (v & !FS_MASK) | ((fs as u8) << 4)).await?;
        self.fs = fs;
        Ok(())
    }

    /// Reboot memory content,
    /// `CTRL_REG5`: `BOOT`
    pub async fn reboot(&mut self, reboot: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG5, BOOT, reboot).await?;
        Ok(())
    }

    /// In boot,
    /// `CTRL_REG5`: `BOOT`
    pub async fn in_boot(&mut self) -> Result<bool, Lis2dh12Error<Spi::Error>> {
        let reg = self.read_reg(Register::CTRL_REG5).await?;
        Ok((reg & BOOT) != 0)
    }

    /// FIFO enable,
    /// `CTRL_REG5`: `FIFO_EN`
    pub async fn enable_fifo(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG5, FIFO_EN, enable).await?;
        Ok(())
    }

    /// Latch interrupt request on INT1_SRC (31h),
    /// with INT1_SRC (31h) register cleared by reading INT1_SRC (31h) itself,
    /// `CTRL_REG5`: `LIR_INT1`
    pub async fn enable_lir_int1(&mut self, latch: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG5, LIR_INT1, latch).await?;
        Ok(())
    }

    /// 4D enable: 4D detection is enabled on INT1 pin
    /// when 6D bit on INT1_CFG (30h) is set to 1,
    /// `CTRL_REG5`: `D4D_INT1`
    pub async fn enable_d4d_int1(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG5, D4D_INT1, enable).await?;
        Ok(())
    }

    /// Latch interrupt request on INT2_SRC (35h) register,
    /// with INT2_SRC (35h) register cleared by reading INT2_SRC (35h) itself,
    /// `CTRL_REG5`: `LIR_INT2`
    pub async fn enable_lir_int2(&mut self, latch: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG5, LIR_INT2, latch).await?;
        Ok(())
    }

    /// 4D enable: 4D detection is enabled on INT2 pin
    /// when 6D bit on INT2_CFG (34h) is set to 1,
    /// `CTRL_REG5`: `D4D_INT2`
    pub async fn enable_d4d_int2(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG5, D4D_INT2, enable).await?;
        Ok(())
    }

    /// `CLICK` interrupt on `INT2` pin,
    /// `CTRL_REG6`: `I2_CLICK`
    pub async fn enable_i2_click(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG6, I2_CLICK, enable).await?;
        Ok(())
    }

    /// `IA1` interrupt on `INT2` pin,
    /// `CTRL_REG6`: `I2_IA1`
    pub async fn enable_i2_ia1(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG6, I2_IA1, enable).await?;
        Ok(())
    }

    /// `IA2` interrupt on `INT2` pin,
    /// `CTRL_REG6`: `I2_IA2`
    pub async fn enable_i2_ia2(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG6, I2_IA2, enable).await?;
        Ok(())
    }

    /// Boot interrupt on `INT2` pin,
    /// `CTRL_REG6`: `I2_BOOT`
    pub async fn enable_i2_boot(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG6, I2_BOOT, enable).await?;
        Ok(())
    }

    /// Activity interrupt on `INT2` pin,
    /// `CTRL_REG6`: `I2_ACT`
    pub async fn enable_i2_act(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG6, I2_ACT, enable).await?;
        Ok(())
    }

    /// INT1/INT2 pin polarity,
    /// `CTRL_REG6`: `INT_POLARITY`
    pub async fn set_int_polarity(&mut self, active_low: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG6, INT_POLARITY, active_low).await?;
        Ok(())
    }

    /// Data status,
    /// `STATUS_REG`: as
    /// DataStatus {zyxor: `ZYXOR`, xyzor: (`XOR`, `YOR`, `ZOR`), zyxda: `ZYXDA`, xyzda: (`XDA`, `YDA`, `ZDA`)}
    pub async fn get_status(&mut self) -> Result<DataStatus, Lis2dh12Error<Spi::Error>> {
        let reg = self.read_reg(Register::STATUS_REG).await?;
        Ok(DataStatus {
            zyxor: (reg & ZYXOR) != 0,
            xyzor: ((reg & XOR) != 0, (reg & YOR) != 0, (reg & ZOR) != 0),
            zyxda: (reg & ZYXDA) != 0,
            xyzda: ((reg & XDA) != 0, (reg & YDA) != 0, (reg & ZDA) != 0),
        })
    }

    /// FIFO mode selection,
    /// `FIFO_CTRL_REG`: `FM`
    pub async fn set_fm(&mut self, fm: FifoMode) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.modify_reg(Register::FIFO_CTRL_REG, |v| {
            (v & !FM_MASK) | ((fm as u8) << 6)
        }).await?;
        Ok(())
    }

    /// FIFO threshold,
    /// `FIFO_CTRL_REG`: `FTH`
    pub async fn set_fth(&mut self, fth: u8) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.modify_reg(Register::FIFO_CTRL_REG, |v| {
            (v & !FTH_MASK) | (fth & FTH_MASK)
        }).await?;
        Ok(())
    }

    /// Disable click interrupt,
    /// `CLICK_CFG` clean all bits
    pub async fn disable_click(&mut self) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.write_reg(Register::CLICK_CFG, 0x00).await?;
        Ok(())
    }

    /// Enable interrupt double-click on X,Y,Z axis,
    /// `CLICK_CFG`: `XD`, `YD`, `ZD`
    pub async fn enable_double_click(
        &mut self,
        (x, y, z): (bool, bool, bool),
    ) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.modify_reg(Register::CLICK_CFG, |mut v| {
            v &= !(XD | YD | ZD); // disable all axises
            v |= if x { XD } else { 0 };
            v |= if y { YD } else { 0 };
            v |= if z { ZD } else { 0 };
            v
        }).await?;
        Ok(())
    }

    /// Enable interrupt single-click on X,Y,Z axis,
    /// `CLICK_CFG`: `XS`, `YS`, `ZS`
    pub async fn enable_single_click(
        &mut self,
        (x, y, z): (bool, bool, bool),
    ) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.modify_reg(Register::CLICK_CFG, |mut v| {
            v &= !(XS | YS | ZS); // disable all axises
            v |= if x { XS } else { 0 };
            v |= if y { YS } else { 0 };
            v |= if z { ZS } else { 0 };
            v
        }).await?;
        Ok(())
    }

    /// Click source,
    /// `CLICK_SRC` decoded as ((`DClick`, `SClick`), `Sign`, (`X`, `Y`, `Z`))
    #[allow(clippy::type_complexity)]
    pub async fn get_click_src(
        &mut self,
    ) -> Result<Option<((bool, bool), bool, (bool, bool, bool))>, Lis2dh12Error<Spi::Error>> {
        let reg = self.read_reg(Register::CLICK_SRC).await?;
        if (reg & IA) != 0 {
            Ok(Some((
                ((reg & DClick) != 0, (reg & SClick) != 0),
                (reg & Sign) != 0,
                ((reg & X) != 0, (reg & Y) != 0, (reg & Z) != 0),
            )))
        } else {
            Ok(None)
        }
    }

    /// If the LIR_Click bit is not set, the interrupt is kept high
    /// for the duration of the latency window.
    /// If the LIR_Click bit is set, the interrupt is kept high
    /// until the CLICK_SRC (39h) register is read.
    /// `CLICK_THS`: `LIR_Click`
    pub async fn enable_lir_click(&mut self, latch: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CLICK_THS, LIR_Click, latch).await?;
        Ok(())
    }

    /// Click threshold,
    /// `CLICK_THS`: `Ths`
    pub async fn set_click_ths(&mut self, ths: u8) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.write_reg(Register::CLICK_THS, ths & THS_MASK).await?;
        Ok(())
    }

    /// Click threshold as f32,
    /// `CLICK_THS`: `Ths`
    pub async fn set_click_thsf(&mut self, ths: f32) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.set_click_ths(self.fs.convert_ths_f32tou8(ths)).await
    }

    /// Click time limit,
    /// `TIME_LIMIT`: `TLI`
    pub async fn set_time_limit(&mut self, tli: u8) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.write_reg(Register::TIME_LIMIT, tli & TLI_MASK).await?;
        Ok(())
    }

    /// Click time latency,
    /// `TIME_LATENCY`: `TLA`
    pub async fn set_time_latency(&mut self, tla: u8) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.write_reg(Register::TIME_LATENCY, tla).await?;
        Ok(())
    }

    /// Click time window,
    /// `TIME_WINDOW`: `TW`
    pub async fn set_time_window(&mut self, tw: u8) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.write_reg(Register::TIME_WINDOW, tw).await?;
        Ok(())
    }

    /// Sleep-to-wake, return-to-sleep activation threshold in low-power mode,
    /// `ACT_THS`: `Acth`
    pub async fn set_act_ths(&mut self, ths: u8) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.write_reg(Register::ACT_THS, ths & Acth_MASK).await?;
        Ok(())
    }

    /// Sleep-to-wake, return-to-sleep activation threshold as f32,
    /// `ACT_THS`: `Acth`
    pub async fn set_act_thsf(&mut self, ths: f32) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.set_act_ths(self.fs.convert_ths_f32tou8(ths)).await
    }

    /// Sleep-to-wake, return-to-sleep duration,
    /// `ACT_DUR`: `ActD`
    pub async fn set_act_dur(&mut self, d: u8) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.write_reg(Register::ACT_DUR, d).await?;
        Ok(())
    }

    /// Temperature sensor enable,
    /// `TEMP_CFG_REG`: `TEMP_EN`,
    /// the `BDU` bit in `CTRL_REG4` is also set
    pub async fn enable_temp(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::TEMP_CFG_REG, TEMP_EN, enable).await?;
        if enable {
            // enable block data update (required for temp reading)
            self.reg_set_bits(Register::CTRL_REG4, BDU).await?;
        }
        Ok(())
    }

    /// Temperature data status,
    /// `STATUS_REG_AUX`: `TOR` - Temperature data overrun,
    ///                   `TDA` - Temperature new data available
    pub async fn get_temp_status(&mut self) -> Result<(bool, bool), Lis2dh12Error<Spi::Error>> {
        let reg = self.read_reg(Register::STATUS_REG_AUX).await?;
        Ok(((reg & TOR) != 0, (reg & TDA) != 0))
    }

    /// Temperature sensor data,
    /// `OUT_TEMP_H`, `OUT_TEMP_L`
    pub async fn get_temp_out(&mut self) -> Result<(i8, u8), Lis2dh12Error<Spi::Error>> {
        let mut buf = [0u8; 3];
        buf[0] = Register::OUT_TEMP_L as u8;
        self.read_bytes(&mut buf).await?;
        Ok((buf[1] as i8, buf[0]))
    }

    /// Temperature sensor data as float,
    /// `OUT_TEMP_H`, `OUT_TEMP_L` converted to `f32`
    pub async fn get_temp_outf(&mut self) -> Result<f32, Lis2dh12Error<Spi::Error>> {
        let (out_h, out_l) = self.get_temp_out().await?;
        // 10-bit resolution
        let value = (i16(out_h) << 8) | i16(out_l);
        Ok(25.0 + f32(value) / 256.0)
    }

    /// `REFERENCE` register
    pub async fn set_ref(&mut self, reference: u8) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.write_reg(Register::REFERENCE, reference).await?;
        Ok(())
    }

    /// `REFERENCE` register
    pub async fn get_ref(&mut self) -> Result<u8, Lis2dh12Error<Spi::Error>> {
        self.read_reg(Register::REFERENCE).await
    }

    /// INT1
    pub async fn int1(&mut self) -> Int<Int1Regs, Spi> {
        Int::new(self)
    }

    /// INT2
    pub async fn int2(&mut self) -> Int<Int2Regs, Spi> {
        Int::new(self)
    }

    /// Resets all registers to their default
    pub async fn reset(&mut self) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.write_reg(Register::CTRL_REG1, CTRL_REG1_DEFAULT).await?;
        self.write_reg(Register::CTRL_REG2, CTRL_REG2_DEFAULT).await?;
        self.write_reg(Register::CTRL_REG3, CTRL_REG3_DEFAULT).await?;
        self.write_reg(Register::CTRL_REG4, CTRL_REG4_DEFAULT).await?;
        self.write_reg(Register::CTRL_REG5, CTRL_REG5_DEFAULT).await?;
        self.write_reg(Register::CTRL_REG6, CTRL_REG6_DEFAULT).await?;
        self.write_reg(Register::INT1_CFG, INT_CFG_DEFAULT).await?;
        self.write_reg(Register::INT2_CFG, INT_CFG_DEFAULT).await?;
        self.write_reg(Register::INT1_THS, INT_THS_DEFAULT).await?;
        self.write_reg(Register::INT2_THS, INT_THS_DEFAULT).await?;
        Ok(())
    }

    /// Enable self test 0, this shouldn't be enabled at the same time as self test 1
    pub async fn enable_st0(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG4, ST0, enable).await?;
        Ok(())
    }

    /// Enable self test 1, this shouldn't be enabled at the same time as self test 0
    pub async fn enable_st1(&mut self, enable: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.reg_xset_bits(Register::CTRL_REG4, ST1, enable).await?;
        Ok(())
    }

    /// Get acceleration reading from the accelerometer
    pub async fn accel_raw(&mut self) -> Result<[i16; 3], Lis2dh12Error<Spi::Error>> {
        let mut buf = [0u8; 7];
        buf[0] = Register::OUT_X_L as u8;
        self.read_bytes(&mut buf).await?;

        Ok([
            (u16(buf[1]) + (u16(buf[2]) << 8)) as i16,
            (u16(buf[3]) + (u16(buf[4]) << 8)) as i16,
            (u16(buf[5]) + (u16(buf[6]) << 8)) as i16
        ])
    }

    /// Get acceleration readin in Gs
    pub async fn accel_norm(&mut self) -> Result<[f32; 3], Lis2dh12Error<Spi::Error>> {
        let acc_raw = self.accel_raw().await?;

        Ok([
            self.fs.convert_out_i16tof32(acc_raw[0]),
            self.fs.convert_out_i16tof32(acc_raw[1]),
            self.fs.convert_out_i16tof32(acc_raw[2]),
        ])
    }

    #[inline]
    async fn read_reg(&mut self, reg: Register) -> Result<u8, Lis2dh12Error<Spi::Error>> {
        let mut buf: [u8; 2] = [reg as u8 | (1 << 6) | (1 << 7), 0];
        self.bus.transfer_in_place(&mut buf[0..2]).await?;
        Ok(buf[1])
    }

    #[inline]
    async fn read_bytes(&mut self, buffer: &mut [u8]) -> Result<(), Lis2dh12Error<Spi::Error>> {
        buffer[0] |= 1 << 7; // set bit 7 to read
        buffer[0] |= 1 << 6; // set bit 6 to automatically increment the index when reading multiple bytes
        self.bus.transfer_in_place(buffer).await?;
        Ok(())
    }

    #[inline]
    async fn write_reg(&mut self, reg: Register, val: u8) -> Result<(), Lis2dh12Error<Spi::Error>> {
        let buf: [u8; 2] = [reg as u8, val];
        self.bus.write(&buf).await?;
        Ok(())
    }

    #[inline]
    async fn modify_reg<F>(&mut self, reg: Register, f: F) -> Result<(), Lis2dh12Error<Spi::Error>>
    where
        F: FnOnce(u8) -> u8,
    {
        let r = self.read_reg(reg).await?;
        self.write_reg(reg, f(r)).await?;
        Ok(())
    }

    #[inline]
    async fn reg_set_bits(&mut self, reg: Register, bits: u8) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.modify_reg(reg, |v| v | bits).await
    }

    #[inline]
    async fn reg_reset_bits(&mut self, reg: Register, bits: u8) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.modify_reg(reg, |v| v & !bits).await
    }

    #[inline]
    async fn reg_xset_bits(&mut self, reg: Register, bits: u8, set: bool) -> Result<(), Lis2dh12Error<Spi::Error>> {
        if set {
            self.reg_set_bits(reg, bits).await
        } else {
            self.reg_reset_bits(reg, bits).await
        }
    }
}


impl<'a, REG, Spi: embedded_hal_async::spi::SpiDevice> Int<'a, REG, Spi>
where
    REG: IntRegs,
{
    fn new(dev: &'a mut Lis2dh12<Spi>) -> Self {
        Self {
            dev,
            reg: PhantomData,
        }
    }

    /// Disable interrupt,
    /// `INTx_CFG` clean all bits
    pub async fn disable(&mut self) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.dev.write_reg(REG::reg_cfg(), 0x00).await?;
        Ok(())
    }

    /// AOI-6D Interrupt mode,
    /// `INTx_CFG`: `AOI`, `6D`
    pub async fn set_mode(&mut self, mode: Aoi6d) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.dev
            .modify_reg(REG::reg_cfg(), |v| (v & !AOI_6D_MASK) | ((mode as u8) << 6)).await?;
        Ok(())
    }

    /// X,Y,Z high event enable,
    /// `INTx_CFG`: `XHIE`, `YHIE`, `ZHIE`
    pub async fn enable_high(&mut self, (x, y, z): (bool, bool, bool)) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.dev.modify_reg(REG::reg_cfg(), |mut v| {
            v &= !(XHIE | YHIE | ZHIE); // disable all axises
            v |= if x { XHIE } else { 0 };
            v |= if y { YHIE } else { 0 };
            v |= if z { ZHIE } else { 0 };
            v
        }).await?;
        Ok(())
    }

    /// X,Y,Z low event enable,
    /// `INTx_CFG`: `XLIE`, `YLIE`, `ZLIE`
    pub async fn enable_low(&mut self, (x, y, z): (bool, bool, bool)) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.dev.modify_reg(REG::reg_cfg(), |mut v| {
            v &= !(XLIE | YLIE | ZLIE); // disable all axises
            v |= if x { XLIE } else { 0 };
            v |= if y { YLIE } else { 0 };
            v |= if z { ZLIE } else { 0 };
            v
        }).await?;
        Ok(())
    }

    /// Source,
    /// `INTx_SRC` decoded as ((`XH`, `XL`), (`YH`, `YL`), (`ZH`, `ZL`))
    #[allow(clippy::type_complexity)]
    pub async fn get_src(
        &mut self,
    ) -> Result<Option<((bool, bool), (bool, bool), (bool, bool))>, Lis2dh12Error<Spi::Error>> {
        let reg = self.dev.read_reg(REG::reg_src()).await?;
        if (reg & IA) != 0 {
            Ok(Some((
                ((reg & XH) != 0, (reg & XL) != 0),
                ((reg & YH) != 0, (reg & YL) != 0),
                ((reg & ZH) != 0, (reg & ZL) != 0),
            )))
        } else {
            Ok(None)
        }
    }

    /// Threshold,
    /// `INTx_THS`: `THS`
    pub async fn set_ths(&mut self, ths: u8) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.dev.write_reg(REG::reg_ths(), ths & THS_MASK).await?;
        Ok(())
    }

    /// Threshold as f32,
    /// `INTx_THS`: `THS`
    pub async fn set_thsf(&mut self, ths: f32) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.set_ths(self.dev.fs.convert_ths_f32tou8(ths)).await
    }

    /// Duration,
    /// `INTx_DURATION`: `D`
    pub async fn set_duration(&mut self, d: u8) -> Result<(), Lis2dh12Error<Spi::Error>> {
        self.dev.write_reg(REG::reg_duration(), d & D_MASK).await?;
        Ok(())
    }
}
