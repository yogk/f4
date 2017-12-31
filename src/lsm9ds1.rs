//! Interfacing the LSM9DS1 3D accelerometer, gyroscope, and magnetometer
//! using SPI.

extern crate stm32f40x;
use stm32f40x::{SPI1, SPI2, SPI3, GPIOA};
use spi::Spi;
use prelude::*;
use math_utils::Vector3;

/////////////////////////////////////////
// Lsm9ds1 Accel/Gyro (XL/G) Registers //
/////////////////////////////////////////
// const ACT_THS: u8 = 0x04;
// const ACT_DUR: u8 = 0x05;
// const INT_GEN_CFG_XL: u8 = 0x06;
// const INT_GEN_THS_X_XL: u8 = 0x07;
// const INT_GEN_THS_Y_XL: u8 = 0x08;
// const INT_GEN_THS_Z_XL: u8 = 0x09;
// const INT_GEN_DUR_XL: u8 = 0x0A;
// const REFERENCE_G: u8 = 0x0B;
// const INT1_CTRL: u8 = 0x0C;
// const INT2_CTRL: u8 = 0x0D;
const WHO_AM_I_XG: u8 = 0x0F;
const CTRL_REG1_G: u8 = 0x10;
const CTRL_REG2_G: u8 = 0x11;
const CTRL_REG3_G: u8 = 0x12;
const ORIENT_CFG_G: u8 = 0x13;
// const INT_GEN_SRC_G: u8 = 0x14;
// const OUT_TEMP_L: u8 = 0x15;
// const OUT_TEMP_H: u8 = 0x16;
// const STATUS_REG_0: u8 = 0x17;
const OUT_X_L_G: u8 = 0x18;
// const OUT_X_H_G: u8 = 0x19;
// const OUT_Y_L_G: u8 = 0x1A;
// const OUT_Y_H_G: u8 = 0x1B;
// const OUT_Z_L_G: u8 = 0x1C;
// const OUT_Z_H_G: u8 = 0x1D;
const CTRL_REG4: u8 = 0x1E;
const CTRL_REG5_XL: u8 = 0x1F;
const CTRL_REG6_XL: u8 = 0x20;
const CTRL_REG7_XL: u8 = 0x21;
const CTRL_REG8: u8 = 0x22;
// const CTRL_REG9: u8 = 0x23;
// const CTRL_REG10: u8 = 0x24;
// const INT_GEN_SRC_XL: u8 = 0x26;
// const STATUS_REG_1: u8 = 0x27;
const OUT_X_L_XL: u8 = 0x28;
// const OUT_X_H_XL: u8 = 0x29;
// const OUT_Y_L_XL: u8 = 0x2A;
// const OUT_Y_H_XL: u8 = 0x2B;
// const OUT_Z_L_XL: u8 = 0x2C;
// const OUT_Z_H_XL: u8 = 0x2D;
// const FIFO_CTRL: u8 = 0x2E;
// const FIFO_SRC: u8 = 0x2F;
// const INT_GEN_CFG_G: u8 = 0x30;
// const INT_GEN_THS_XH_G: u8 = 0x31;
// const INT_GEN_THS_XL_G: u8 = 0x32;
// const INT_GEN_THS_YH_G: u8 = 0x33;
// const INT_GEN_THS_YL_G: u8 = 0x34;
// const INT_GEN_THS_ZH_G: u8 = 0x35;
// const INT_GEN_THS_ZL_G: u8 = 0x36;
// const INT_GEN_DUR_G: u8 = 0x37;

///////////////////////////////
// Lsm9ds1 Magneto Registers //
///////////////////////////////
// const OFFSET_X_REG_L_M: u8 = 0x05;
// const OFFSET_X_REG_H_M: u8 = 0x06;
// const OFFSET_Y_REG_L_M: u8 = 0x07;
// const OFFSET_Y_REG_H_M: u8 = 0x08;
// const OFFSET_Z_REG_L_M: u8 = 0x09;
// const OFFSET_Z_REG_H_M: u8 = 0x0A;
const WHO_AM_I_M: u8 = 0x0F;
const CTRL_REG1_M: u8 = 0x20;
const CTRL_REG2_M: u8 = 0x21;
const CTRL_REG3_M: u8 = 0x22;
const CTRL_REG4_M: u8 = 0x23;
const CTRL_REG5_M: u8 = 0x24;
// const STATUS_REG_M: u8 = 0x27;
const OUT_X_L_M: u8 = 0x28;
// const OUT_X_H_M: u8 = 0x29;
// const OUT_Y_L_M: u8 = 0x2A;
// const OUT_Y_H_M: u8 = 0x2B;
// const OUT_Z_L_M: u8 = 0x2C;
// const OUT_Z_H_M: u8 = 0x2D;
// const INT_CFG_M: u8 = 0x30;
// const INT_SRC_M: u8 = 0x31;
// const INT_THS_L_M: u8 = 0x32;
// const INT_THS_H_M: u8 = 0x33;

////////////////////////////////
// Lsm9ds1 WHO_AM_I Responses //
////////////////////////////////
const WHO_AM_I_AG_RSP: u8 = 0x68;
const WHO_AM_I_M_RSP: u8 = 0x3D;

/// Accelerometer settings
pub struct AccelSettings {
    ///
    pub enabled: bool,
    /// accel scale can be 2, 4, 8, or 16
    pub scale: u16,
    /// accel sample rate can be 1-6
    /// 1 = 10 Hz    4 = 238 Hz
    /// 2 = 50 Hz    5 = 476 Hz
    /// 3 = 119 Hz   6 = 952 Hz
    pub sample_rate: u8,
    ///
    pub enable_x: bool,
    ///
    pub enable_y: bool,
    ///
    pub enable_z: bool,
    /// Accel cutoff freqeuncy can be any value between -1 - 3.
    /// -1 = bandwidth determined by sample rate
    /// 0 = 408 Hz   2 = 105 Hz
    /// 1 = 211 Hz   3 = 50 Hz
    pub bandwidth: i8,
    ///
    pub high_res_enable: bool,
    /// accelHighResBandwidth can be any value between 0-3
    /// LP cutoff is set to a factor of sample rate
    /// 0 = ODR/50    2 = ODR/9
    /// 1 = ODR/100   3 = ODR/400
    pub high_res_bandwidth: u8,
}

/// Gyroscope settings
pub struct GyroSettings {
    ///
    pub enabled: bool,
    /// gyro scale can be 245, 500, or 2000
    pub scale: u16,
    /// gyro sample rate: value between 1-6
    /// 1 = 14.9    4 = 238
    /// 2 = 59.5    5 = 476
    /// 3 = 119     6 = 952
    pub sample_rate: u8,
    /// gyro cutoff frequency: value between 0-3
    /// Actual value of cutoff frequency depends
    /// on sample rate.
    pub bandwidth: u8,
    ///
    pub low_power_enable: bool,
    ///
    pub hpf_enable: bool,
    ///  Gyro HPF cutoff frequency: value between 0-9
    /// Actual value depends on sample rate. Only applies
    /// if gyroHPFEnable is true.
    pub hpf_cutoff: u8,
    ///
    pub flip_x: bool,
    ///
    pub flip_y: bool,
    ///
    pub flip_z: bool,
    ///
    pub orientation: u8,
    ///
    pub enable_x: bool,
    ///
    pub enable_y: bool,
    ///
    pub enable_z: bool,
    ///
    pub latch_interrupt: bool,
}

/// Magnetometer settings
pub struct MagSettings {
    ///
    pub enabled: bool,
    /// mag scale can be 4, 8, 12, or 16
    pub scale: u8,
    ///mag data rate can be 0-7
    /// 0 = 0.625 Hz  4 = 10 Hz
    /// 1 = 1.25 Hz   5 = 20 Hz
    /// 2 = 2.5 Hz    6 = 40 Hz
    /// 3 = 5 Hz      7 = 80 Hz
    pub sample_rate: u8,
    ///
    pub temp_compensation_enable: bool,
    /// magPerformance can be any value between 0-3
    /// 0 = Low power mode      2 = high performance
    /// 1 = medium performance  3 = ultra-high performance
    pub xy_performance: u8,
    ///
    pub z_performance: u8,
    ///
    pub low_power_enable: bool,
    /// magOperatingMode can be 0-2
    /// 0 = continuous conversion
    /// 1 = single-conversion
    /// 2 = power down
    pub operating_mode: u8,
}

/// Settings for lsm9ds1
pub struct ImuSettings {
    ///
    pub mag: MagSettings,
    ///
    pub accel: AccelSettings,
    ///
    pub gyro: GyroSettings,
}

/// Default settings for lsm9ds1
impl ImuSettings {
    ///
    pub const fn new() -> Self {
        ImuSettings {
            accel: AccelSettings {
                enabled: true,
                scale: 2,
                sample_rate: 3,
                enable_x: true,
                enable_y: true,
                enable_z: true,
                bandwidth: -1,
                high_res_enable: false,
                high_res_bandwidth: 0,
            },
            gyro: GyroSettings {
                enabled: true,
                scale: 500,
                sample_rate: 3,
                bandwidth: 0,
                low_power_enable: false,
                hpf_enable: false,
                hpf_cutoff: 0,
                flip_x: false,
                flip_y: false,
                flip_z: false,
                orientation: 0,
                enable_x: true,
                enable_y: true,
                enable_z: true,
                latch_interrupt: true,
            },
            mag: MagSettings {
                enabled: true,
                scale: 4,
                sample_rate: 7,
                temp_compensation_enable: false,
                xy_performance: 1,
                z_performance: 1,
                low_power_enable: false,
                operating_mode: 0,
            },
        }
    }
}

const SENSITIVITY_ACCELEROMETER_2: f32 = 0.000061;
const SENSITIVITY_ACCELEROMETER_4: f32 = 0.000122;
const SENSITIVITY_ACCELEROMETER_8: f32 = 0.000244;
const SENSITIVITY_ACCELEROMETER_16: f32 = 0.000732;
const SENSITIVITY_GYROSCOPE_245: f32 = 0.00875;
const SENSITIVITY_GYROSCOPE_500: f32 = 0.0175;
const SENSITIVITY_GYROSCOPE_2000: f32 = 0.07;
const SENSITIVITY_MAGNETOMETER_4: f32 = 0.00014;
const SENSITIVITY_MAGNETOMETER_8: f32 = 0.00029;
const SENSITIVITY_MAGNETOMETER_12: f32 = 0.00043;
const SENSITIVITY_MAGNETOMETER_16: f32 = 0.00058;

/// LSM9DS1 for SPI communication
pub struct Lsm9ds1<'a, T>(pub &'a T)
where
    T: 'a;

macro_rules! impl_Lsm9ds1 {
    ($SPI:ident) => {
        impl<'a> Lsm9ds1<'a, $SPI> {

            fn enable_m(&self, gpioa: &GPIOA) {
                gpioa.odr.modify(|_, w| w.odr9().bit(false));
            }

            fn disable_m(&self, gpioa: &GPIOA) {
                gpioa.odr.modify(|_, w| w.odr9().bit(true));
            }

            fn enable_ag(&self, gpioa: &GPIOA) {
                gpioa.odr.modify(|_, w| w.odr8().bit(false));
            }

            fn disable_ag(&self, gpioa: &GPIOA) {
                gpioa.odr.modify(|_, w| w.odr8().bit(true));
            }

            fn write_bytes(&self, spi: &Spi<$SPI>, addr: u8, data: [u8; 128], length: u8) {
                let mut x: u8 = 0;
                while spi.send(addr).is_err() {}
                while spi.read().is_err() {}
                while x < length {
                    while spi.send(data[x as usize]).is_err() {}
                    while spi.read().is_err() {}
                    x = x + 1;
                }
            }

            fn read_bytes(&self, spi: &Spi<$SPI>, addr: u8, length: u8) -> [u8; 128] {
                let mut response: [u8; 128] = [0; 128];
                let mut x: u8 = 0;
                while spi.send(addr).is_err() {}
                while spi.read().is_err() {}
                while x < length {
                    while spi.send(0).is_err() {}
                    loop {
                        if let Ok(byte) = spi.read() {
                            response[x as usize] = byte;
                            break;
                        }
                    }
                    x = x + 1;
                }
                response
            }

            fn ag_read_bytes(&self, spi: &Spi<$SPI>, gpioa: &GPIOA, addr: u8, length: u8) -> [u8; 128] {
                let addr = 0x80 | (addr & 0x7F);
                self.enable_ag(&gpioa);
                let ans = self.read_bytes(spi, addr, length);
                self.disable_ag(&gpioa);
                ans
            }

            fn ag_read_byte(&self, spi: &Spi<$SPI>, gpioa: &GPIOA, addr: u8) -> u8 {
                self.ag_read_bytes(spi,gpioa,addr,1)[0]
            }

            fn ag_write_bytes(&self, spi: &Spi<$SPI>, gpioa: &GPIOA, addr: u8, data: [u8; 128], length: u8) {
                let addr = !0x80 & (addr & 0x7F);
                self.enable_ag(&gpioa);
                self.write_bytes(spi, addr, data, length);
                self.disable_ag(&gpioa);
            }

            fn ag_write_byte(&self, spi: &Spi<$SPI>, gpioa: &GPIOA, addr: u8, data: u8) {
                self.ag_write_bytes(spi, gpioa, addr, [data; 128], 1);
            }

            fn m_read_bytes(&self, spi: &Spi<$SPI>, gpioa: &GPIOA, addr: u8, length: u8) -> [u8; 128] {
                let addr = 0x80 | 0x40 | (addr & 0x3F);
                self.enable_m(&gpioa);
                let ans = self.read_bytes(spi, addr, length);
                self.disable_m(&gpioa);
                ans
            }

            fn m_read_byte(&self, spi: &Spi<$SPI>, gpioa: &GPIOA, addr: u8) -> u8 {
                self.m_read_bytes(spi,gpioa,addr,1)[0]
            }

            fn m_write_bytes(&self, spi: &Spi<$SPI>, gpioa: &GPIOA, addr: u8, data: [u8; 128], length: u8) {
                let addr = !0x80 & (0x40 | addr & 0x7F);
                self.enable_m(&gpioa);
                self.write_bytes(spi, addr, data, length);
                self.disable_m(&gpioa);
            }

            fn m_write_byte(&self, spi: &Spi<$SPI>, gpioa: &GPIOA, addr: u8, data: u8) {
                self.m_write_bytes(spi, gpioa, addr, [data; 128], 1);
            }

            /// Resets and reboots the accelerometer, gyro and magnetometer.
            pub fn reset(&self, spi: &Spi<$SPI>, gpioa: &GPIOA) {
                // Check that we can read from acc/gyro
                assert_eq!(WHO_AM_I_AG_RSP, self.ag_read_byte(&spi, &gpioa, WHO_AM_I_XG));
                self.ag_write_byte(&spi, &gpioa, CTRL_REG8, 0b10000001);
                loop {
                    if self.ag_read_byte(&spi, &gpioa, WHO_AM_I_XG) == WHO_AM_I_AG_RSP {
                        break;
                    }
                }
                // Check that we can read from mag
                assert_eq!(WHO_AM_I_M_RSP, self.m_read_byte(&spi, &gpioa, WHO_AM_I_M));
                self.m_write_byte(&spi, &gpioa, CTRL_REG2_M, 0b00001100);
                loop {
                    if self.m_read_byte(&spi, &gpioa, WHO_AM_I_M) == WHO_AM_I_M_RSP {
                        break;
                    }
                }
            }

            /// Initalizes the gyroscope with given settings
            pub fn init_gyro(&self, spi: &Spi<$SPI>, gpioa: &GPIOA, settings: &ImuSettings) {
                // Check that we can read from acc/gyro
                assert_eq!(WHO_AM_I_AG_RSP, self.ag_read_byte(&spi, &gpioa, WHO_AM_I_XG));

                let mut temp: u8 = 0;
                // CTRL_REG1_G (Default value: 0x00)
                // [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
                // ODR_G[2:0] - Output data rate selection
                // FS_G[1:0] - Gyroscope full-scale selection
                // BW_G[1:0] - Gyroscope bandwidth selection

                // To disable gyro, set sample rate bits to 0. We'll only set sample
                // rate if the gyro is enabled.
                if settings.gyro.enabled {
                    temp = (settings.gyro.sample_rate & 0x07) << 5;
                }
                temp |= match settings.gyro.scale {
                    500 => (0x1 << 3),
                    2000 => (0x3 << 3),
                    _ => 0, // Otherwise we'll set it to 245 dps (0x0 << 4)
                };
                temp |= settings.gyro.bandwidth & 0x3;
                self.ag_write_byte(&spi, &gpioa, CTRL_REG1_G, temp);

                // Check that we can both read and write
                assert_eq!(temp, self.ag_read_byte(&spi, &gpioa, CTRL_REG1_G));

                // CTRL_REG2_G (Default value: 0x00)
                // [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
                // INT_SEL[1:0] - INT selection configuration
                // OUT_SEL[1:0] - Out selection configuration
                self.ag_write_byte(&spi, &gpioa, CTRL_REG2_G, 0x00);
                assert_eq!(0x00, self.ag_read_byte(&spi, &gpioa, CTRL_REG2_G));

                // CTRL_REG3_G (Default value: 0x00)
                // [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
                // LP_mode - Low-power mode enable (0: disabled, 1: enabled)
                // HP_EN - HPF enable (0:disabled, 1: enabled)
                // HPCF_G[3:0] - HPF cutoff frequency
                temp = 0;
                if settings.gyro.low_power_enable {
                    temp = 1 << 7;
                }
                if settings.gyro.hpf_enable {
                    temp |= (1 << 6) | (settings.gyro.hpf_cutoff & 0x0F);
                }
                self.ag_write_byte(&spi, &gpioa, CTRL_REG3_G, temp);
                assert_eq!(temp, self.ag_read_byte(&spi, &gpioa, CTRL_REG3_G));

                // CTRL_REG4 (Default value: 0x38)
                // [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
                // Zen_G - Z-axis output enable (0:disable, 1:enable)
                // Yen_G - Y-axis output enable (0:disable, 1:enable)
                // Xen_G - X-axis output enable (0:disable, 1:enable)
                // LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
                // 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
                temp = 0;
                if settings.gyro.enable_z {
                    temp |= 1 << 5
                }
                if settings.gyro.enable_y {
                    temp |= 1 << 4
                }
                if settings.gyro.enable_x {
                    temp |= 1 << 3
                }
                if settings.gyro.latch_interrupt {
                    temp |= 1 << 1
                }
                self.ag_write_byte(&spi, &gpioa, CTRL_REG4, temp);
                assert_eq!(temp, self.ag_read_byte(&spi, &gpioa, CTRL_REG4));

                // ORIENT_CFG_G (Default value: 0x00)
                // [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
                // SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
                // Orient [2:0] - Directional user orientation selection
                temp = 0;
                if settings.gyro.flip_x {
                    temp |= 1 << 5
                }
                if settings.gyro.flip_y {
                    temp |= 1 << 4
                }
                if settings.gyro.flip_z {
                    temp |= 1 << 3
                }
                self.ag_write_byte(&spi, &gpioa, ORIENT_CFG_G, temp);
                assert_eq!(temp, self.ag_read_byte(&spi, &gpioa, ORIENT_CFG_G));
            }

            /// Reads the raw values from the gyro X/Y/Z axis
            pub fn read_gyro_raw(&self,spi: &Spi<$SPI>, gpioa: &GPIOA) -> Vector3<i16> {
                // Read 6 bytes, beginning at OUT_X_L_G
                let temp = self.ag_read_bytes(&spi, &gpioa, OUT_X_L_G, 6);
                Vector3 {
                    x:(((temp[1] as u16) << 8) | temp[0] as u16) as i16,
                    y:(((temp[3] as u16) << 8) | temp[2] as u16) as i16,
                    z:(((temp[5] as u16) << 8) | temp[4] as u16) as i16,
                }
            }

            /// Reads the values from the gyro X/Y/Z axis in DPS
            pub fn read_gyro(&self,spi: &Spi<$SPI>, gpioa: &GPIOA, settings: &ImuSettings) -> Vector3<f32> {
                let g_res = match settings.gyro.scale {
                    245 => SENSITIVITY_GYROSCOPE_245,
                    500 => SENSITIVITY_GYROSCOPE_500,
                    2000 => SENSITIVITY_GYROSCOPE_2000,
                    _ => panic!("Invalid sensitivity")
                } / 2.0;
                let g_raw = self.read_gyro_raw(&spi, &gpioa);
                Vector3 {
                    x:g_raw.x as f32 * g_res,
                    y:g_raw.y as f32 * g_res,
                    z:g_raw.z as f32 * g_res,
                }
            }

            /// Initalizes the accelerometer with given settings
            pub fn init_acc(&self, spi: &Spi<$SPI>, gpioa: &GPIOA, settings: &ImuSettings) {
                // Check that we can read from acc/gyro
                assert_eq!(WHO_AM_I_AG_RSP, self.ag_read_byte(&spi, &gpioa, WHO_AM_I_XG));

                let mut temp: u8 = 0;

                //	CTRL_REG5_XL (0x1F) (Default value: 0x38)
                //	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
                //	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
                //		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
                //	Zen_XL - Z-axis output enabled
                //	Yen_XL - Y-axis output enabled
                //	Xen_XL - X-axis output enabled
                if settings.accel.enable_z {temp |= 1<<5};
                if settings.accel.enable_y {temp |= 1<<4};
                if settings.accel.enable_x {temp |= 1<<3};

                self.ag_write_byte(&spi, &gpioa, CTRL_REG5_XL, temp);
                assert_eq!(temp, self.ag_read_byte(&spi, &gpioa, CTRL_REG5_XL));

                // CTRL_REG6_XL (0x20) (Default value: 0x00)
                // [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
                // ODR_XL[2:0] - Output data rate & power mode selection
                // FS_XL[1:0] - Full-scale selection
                // BW_SCAL_ODR - Bandwidth selection
                // BW_XL[1:0] - Anti-aliasing filter bandwidth selection
                temp = 0;
                // To disable the accel, set the sampleRate bits to 0.
                if settings.accel.enabled
                {
                    temp |= (settings.accel.sample_rate & 0x07) << 5;
                }

                temp |= match settings.accel.scale {
                    4 => (0x2 << 3),
                    8 => (0x3 << 3),
                    16 => (0x1 << 3),
                    _ => 0, // Otherwise it'll be set to 2g (0x0 << 3)
                };

                if settings.accel.bandwidth >= 0 {
                    temp |= 1<<2; // Set BW_SCAL_ODR
                    temp |= settings.accel.bandwidth as u8 & 0x03;
                }
                self.ag_write_byte(&spi, &gpioa, CTRL_REG6_XL, temp);
                assert_eq!(temp, self.ag_read_byte(&spi, &gpioa, CTRL_REG6_XL));

                // CTRL_REG7_XL (0x21) (Default value: 0x00)
                // [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
                // HR - High resolution mode (0: disable, 1: enable)
                // DCF[1:0] - Digital filter cutoff frequency
                // FDS - Filtered data selection
                // HPIS1 - HPF enabled for interrupt function
                temp = 0;
                if settings.accel.high_res_enable
                {
                    temp |= 1<<7; // Set HR bit
                    temp |= (settings.accel.high_res_bandwidth & 0x3) << 5;
                }
                self.ag_write_byte(&spi, &gpioa, CTRL_REG7_XL, temp);
                assert_eq!(temp, self.ag_read_byte(&spi, &gpioa, CTRL_REG7_XL));
            }

            /// Reads the raw values from the accelerometer X/Y/Z axis
            pub fn read_acc_raw(&self,spi: &Spi<$SPI>, gpioa: &GPIOA) -> Vector3<i16> {
                // Read 6 bytes, beginning at OUT_X_L_XL
                let temp = self.ag_read_bytes(&spi, &gpioa, OUT_X_L_XL, 6);
                Vector3 {
                    x:(((temp[1] as u16) << 8) | temp[0] as u16) as i16,
                    y:(((temp[3] as u16) << 8) | temp[2] as u16) as i16,
                    z:(((temp[5] as u16) << 8) | temp[4] as u16) as i16,
                }
            }

            /// Reads the values from the accelerometer X/Y/Z axis in g's
            pub fn read_acc(&self,spi: &Spi<$SPI>, gpioa: &GPIOA, settings: &ImuSettings) -> Vector3<f32> {
                let a_res = match settings.accel.scale {
                    2 => SENSITIVITY_ACCELEROMETER_2,
                    4=> SENSITIVITY_ACCELEROMETER_4,
                    8=> SENSITIVITY_ACCELEROMETER_8,
                    16=> SENSITIVITY_ACCELEROMETER_16,
                    _ => panic!("Invalid sensitivity"),
                };
                let a_raw = self.read_acc_raw(&spi, &gpioa);
                Vector3 {
                    x:a_raw.x as f32 * a_res,
                    y:a_raw.y as f32 * a_res,
                    z:a_raw.z as f32 * a_res,
                }
            }

            /// Initalizes the magnetometer with given settings
            pub fn init_mag(&self, spi: &Spi<$SPI>, gpioa: &GPIOA, settings: &ImuSettings) -> bool {
                // Check that we can read from acc/gyro
                assert_eq!(WHO_AM_I_M_RSP, self.m_read_byte(&spi, &gpioa, WHO_AM_I_M));

                let mut temp: u8 = 0;

                // CTRL_REG1_M (Default value: 0x10)
                // [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
                // TEMP_COMP - Temperature compensation
                // OM[1:0] - X & Y axes op mode selection
                //	00:low-power, 01:medium performance
                //	10: high performance, 11:ultra-high performance
                // DO[2:0] - Output data rate selection
                // ST - Self-test enable
                if settings.mag.temp_compensation_enable {temp |= 1<<7};
                temp |= (settings.mag.xy_performance & 0x3) << 5;
                temp |= (settings.mag.sample_rate & 0x7) << 2;
                self.m_write_byte(&spi, &gpioa, CTRL_REG1_M, temp);
                assert_eq!(temp, self.m_read_byte(&spi, &gpioa, CTRL_REG1_M));

                // CTRL_REG2_M (Default value 0x00)
                // [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
                // FS[1:0] - Full-scale configuration
                // REBOOT - Reboot memory content (0:normal, 1:reboot)
                // SOFT_RST - Reset config and user registers (0:default, 1:reset)
                temp = 0;
                temp |= match settings.mag.scale
                {
                 8 => 0x1 << 5,
                12 => 0x2 << 5,
                16 => 0x3 << 5,
                _ => 0,
                // Otherwise we'll default to 4 gauss (00)
                };
                self.m_write_byte(&spi, &gpioa, CTRL_REG2_M, temp); // +/-4Gauss
                assert_eq!(temp, self.m_read_byte(&spi, &gpioa, CTRL_REG2_M));

                // CTRL_REG3_M (Default value: 0x03)
                // [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
                // I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
                // LP - Low-power mode cofiguration (1:enable)
                // SIM - SPI mode selection (0:write-only, 1:read/write enable)
                // MD[1:0] - Operating mode
                //	00:continuous conversion, 01:single-conversion,
                //  10,11: Power-down
                temp = 0;
                if settings.mag.low_power_enable {temp |= 1<<5};
                temp |= settings.mag.operating_mode & 0x3;
                self.m_write_byte(&spi, &gpioa, CTRL_REG3_M, temp); // Continuous conversion mode
                assert_eq!(temp, self.m_read_byte(&spi, &gpioa, CTRL_REG3_M));

                // CTRL_REG4_M (Default value: 0x00)
                // [0][0][0][0][OMZ1][OMZ0][BLE][0]
                // OMZ[1:0] - Z-axis operative mode selection
                //	00:low-power mode, 01:medium performance
                //	10:high performance, 11:ultra-high performance
                // BLE - Big/little endian data
                temp = (settings.mag.z_performance & 0x3) << 2;
                self.m_write_byte(&spi, &gpioa, CTRL_REG4_M, temp);
                assert_eq!(temp, self.m_read_byte(&spi, &gpioa, CTRL_REG4_M));

                // CTRL_REG5_M (Default value: 0x00)
                // [0][BDU][0][0][0][0][0][0]
                // BDU - Block data update for magnetic data
                //	0:continuous, 1:not updated until MSB/LSB are read
                temp = 0;
                self.m_write_byte(&spi, &gpioa, CTRL_REG5_M, temp);
                assert_eq!(temp, self.m_read_byte(&spi, &gpioa, CTRL_REG5_M));

                true
            }

            /// Reads the raw values from the magnetometer X/Y/Z axis
            pub fn read_mag_raw(&self,spi: &Spi<$SPI>, gpioa: &GPIOA) -> Vector3<i16> {

                // Read 6 bytes, beginning at OUT_X_L_M
                let temp = self.m_read_bytes(&spi, &gpioa, OUT_X_L_M, 6);
                Vector3 {
                    x:(((temp[1] as u16) << 8) | temp[0] as u16) as i16,
                    y:(((temp[3] as u16) << 8) | temp[2] as u16) as i16,
                    z:(((temp[5] as u16) << 8) | temp[4] as u16) as i16,
                }
            }

            /// Reads the values from the magnetometer X/Y/Z axis in Gauss
            pub fn read_mag(&self,spi: &Spi<$SPI>, gpioa: &GPIOA, settings: &ImuSettings) -> Vector3<f32> {
                let m_res = match settings.mag.scale {
                    4 => SENSITIVITY_MAGNETOMETER_4,
                    8 => SENSITIVITY_MAGNETOMETER_8,
                    12 => SENSITIVITY_MAGNETOMETER_12,
                    16 => SENSITIVITY_MAGNETOMETER_16,
                    _ => panic!("Invalid sensitivity")
                };
                let m_raw = self.read_mag_raw(&spi, &gpioa);
                Vector3 {
                    x:m_raw.x as f32 * m_res,
                    y:m_raw.y as f32 * m_res,
                    z:m_raw.z as f32 * m_res,
                }
            }

        }
    }
}

impl_Lsm9ds1!(SPI1);
impl_Lsm9ds1!(SPI2);
impl_Lsm9ds1!(SPI3);
