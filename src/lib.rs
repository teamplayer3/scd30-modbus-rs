#![cfg_attr(not(feature = "std"), no_std)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]

use core::time::Duration;

use embedded_hal::serial::{Read, Write};
use smol::Timer;

#[derive(Debug)]
pub enum Error<E> {
    WrongCrc,
    Serial(E),
    GotWrongResponse,
}

#[cfg(feature = "std")]
impl<E: std::fmt::Display> std::fmt::Display for Error<E> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Serial(e) => write!(f, "Serial error: {}", e),
            Self::GotWrongResponse => write!(f, "Wrong response"),
            Self::WrongCrc => write!(f, "Wrong checksum"),
        }
    }
}

#[cfg(feature = "std")]
impl<E: std::error::Error> std::error::Error for Error<E> {}

pub enum Command {
    StartContinuousMeasurement = 0x0036,
    StopContinuousMeasurement = 0x0037,
    SetMeasurementInterval = 0x0025,
    GetDataReadyStatus = 0x0027,
    ReadMeasurement = 0x0028,
    SetAutomaticSelfCalibration = 0x003A,
    ForcedRecalibrationValue = 0x0039,
    SetTemperatureOffset = 0x003B,
    SetAltitude = 0x0038,
    ReadFirmwareVersion = 0x0020,
    SoftReset = 0x0034,
}

pub enum Function {
    ReadHoldingReg = 0x3,
    ReadInputReg = 0x4,
    WriteHoldingReg = 0x6,
}

impl From<Function> for u8 {
    fn from(f: Function) -> Self {
        f as u8
    }
}

const RESPONSE_TIME: Duration = Duration::from_millis(5);
const MODBUS_ADDR: u8 = 0x61;

pub struct Scd30<Serial> {
    serial: Serial,
}

#[derive(Debug)]
pub struct Measurement {
    pub co2: f32,
    pub humidity: f32,
    pub temperature: f32,
}

/// See the [datasheet] for I²c parameters.
///
/// [datasheet]: https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/9.5_CO2/Sensirion_CO2_Sensors_SCD30_Interface_Description.pdf
impl<Serial, E> Scd30<Serial>
where
    Serial: Read<u8, Error = E> + Write<u8, Error = E>,
{
    pub const BAUDRATE: u16 = 19200;

    /// Returns an [Scd30] instance with the default address 0x61 shifted one place to the left.
    /// You may or may not need this bitshift depending on the byte size of
    /// your [I²c](embedded_hal::blocking::i2c) peripheral.
    pub fn new(mut serial: Serial) -> Self {
        for _ in 0..5 {
            let _ = nb::block!(serial.read()).map_err(Error::Serial);
        }
        Scd30 { serial }
    }

    pub async fn soft_reset(&mut self) -> Result<(), Error<E>> {
        self.write(Function::WriteHoldingReg.into(), Command::SoftReset, 0x0001)?;
        if !self.check_resp_data::<6>(4, 0x0001).await? {
            return Err(Error::GotWrongResponse);
        }
        Ok(())
    }

    pub fn stop_measuring(&mut self) -> Result<(), Error<E>> {
        self.write(
            Function::WriteHoldingReg.into(),
            Command::StopContinuousMeasurement,
            0x001,
        )
    }

    /// Enable or disable automatic self calibration (ASC).
    ///
    /// According to the datasheet, the sensor should be active continously for at least
    /// 7 days to find the initial parameter for ASC. The sensor has to be exposed to
    /// at least 1 hour of fresh air (~400ppm CO₂) per day.
    pub async fn set_automatic_calibration(&mut self, enable: bool) -> Result<(), Error<E>> {
        let data = match enable {
            true => 0x0001,
            _ => 0,
        };
        self.write(
            Function::ReadHoldingReg.into(),
            Command::SetAutomaticSelfCalibration,
            data,
        )?;
        if !self.check_resp_data::<5>(3, data).await? {
            return Err(Error::GotWrongResponse);
        }
        Ok(())
    }

    /// Set forced recalibration value (FCR).
    ///
    /// CO2 reference must be in unit ppm.
    pub async fn force_recalibrate_with_value(&mut self, reference: u16) -> Result<(), Error<E>> {
        self.write(
            Function::WriteHoldingReg.into(),
            Command::ForcedRecalibrationValue,
            reference,
        )?;
        if !self.check_resp_data::<6>(4, reference).await? {
            return Err(Error::GotWrongResponse);
        }
        Ok(())
    }

    pub async fn get_forced_recalibration_value(&mut self) -> Result<u16, Error<E>> {
        self.write(
            Function::ReadHoldingReg.into(),
            Command::ForcedRecalibrationValue,
            0x0001,
        )?;
        Timer::after(RESPONSE_TIME).await;
        let resp = self.read_n::<5>()?;
        Ok(u16::from_be_bytes([resp[3], resp[4]]))
    }

    pub async fn set_temperature_offset(&mut self, offset: u16) -> Result<(), Error<E>> {
        self.write(
            Function::WriteHoldingReg.into(),
            Command::SetTemperatureOffset,
            offset,
        )?;
        if !self.check_resp_data::<6>(4, offset).await? {
            return Err(Error::GotWrongResponse);
        }
        Ok(())
    }

    /// Start measuring without mbar compensation.
    pub async fn start_measuring(&mut self) -> Result<(), Error<E>> {
        self.start_measuring_with_mbar(0).await
    }

    pub async fn set_measurement_interval(&mut self, interval: Duration) -> Result<(), Error<E>> {
        let secs = interval.as_secs();
        debug_assert!(secs >= 2 && secs <= 1800);
        let secs = u16::try_from(secs).unwrap();
        self.write(
            Function::WriteHoldingReg.into(),
            Command::SetMeasurementInterval,
            secs,
        )?;
        if !self.check_resp_data::<6>(4, secs).await? {
            return Err(Error::GotWrongResponse);
        }
        Ok(())
    }

    /// Start measuring with mbar (pressure) compensation.
    pub async fn start_measuring_with_mbar(&mut self, pressure: u16) -> Result<(), Error<E>> {
        debug_assert!(pressure == 0 || (pressure >= 700 && pressure <= 1400));
        self.write(
            Function::WriteHoldingReg.into(),
            Command::StartContinuousMeasurement,
            pressure,
        )?;
        if !self.check_resp_data::<6>(4, pressure).await? {
            return Err(Error::GotWrongResponse);
        }
        Ok(())
    }

    pub async fn data_ready(&mut self) -> Result<bool, Error<E>> {
        self.write(
            Function::ReadHoldingReg.into(),
            Command::GetDataReadyStatus,
            0x0001,
        )?;
        self.check_resp_data::<5>(3, 0x0001).await
    }

    pub async fn read(&mut self) -> Result<Option<Measurement>, Error<E>> {
        match self.data_ready().await {
            Ok(true) => {
                self.write(
                    Function::ReadHoldingReg.into(),
                    Command::ReadMeasurement,
                    0x0006,
                )?;
                let buffer = self.read_n::<15>()?;
                Ok(Some(Measurement {
                    co2: f32::from_bits(u32::from_be_bytes([
                        buffer[3], buffer[4], buffer[5], buffer[6],
                    ])),
                    temperature: f32::from_bits(u32::from_be_bytes([
                        buffer[7], buffer[8], buffer[9], buffer[10],
                    ])),
                    humidity: f32::from_bits(u32::from_be_bytes([
                        buffer[11], buffer[12], buffer[13], buffer[14],
                    ])),
                }))
            }
            Ok(false) => Ok(None),
            Err(e) => Err(e),
        }
    }

    async fn check_resp_data<const N: usize>(
        &mut self,
        high_byte_index: usize,
        evaluate_data: u16,
    ) -> Result<bool, Error<E>>
    where
        [(); N + 2]: Sized,
    {
        Timer::after(RESPONSE_TIME).await;
        let resp = self.read_n::<N>()?;
        Ok(u16::from_be_bytes([resp[high_byte_index], resp[high_byte_index + 1]]) == evaluate_data)
    }

    fn read_n<const N: usize>(&mut self) -> Result<[u8; N], Error<E>>
    where
        [(); N + 2]: Sized,
    {
        let mut buffer = [0u8; N + 2];
        for i in 0..(N + 2) {
            let b = nb::block!(self.serial.read()).map_err(Error::Serial)?;
            buffer[i] = b;
        }

        if !check_crc(&buffer) {
            return Err(Error::WrongCrc);
        }

        let mut output = [0u8; N];
        output.copy_from_slice(&buffer[..N]);

        Ok(output)
    }

    fn write(&mut self, func: u8, cmd: Command, data: u16) -> Result<(), Error<E>> {
        let mut buffer = [0u8; 6 + 2];

        buffer[0] = MODBUS_ADDR;
        buffer[1] = func;

        buffer[2..=3].copy_from_slice(&(cmd as u16).to_be_bytes());
        buffer[4..=5].copy_from_slice(&data.to_be_bytes());

        let crc = calculate_crc(&buffer[..6]);
        buffer[6..=7].copy_from_slice(&crc.to_be_bytes());
        self.write_n(&buffer)
    }

    fn write_n(&mut self, data: &[u8]) -> Result<(), Error<E>> {
        for b in data {
            nb::block!(self.serial.write(*b)).map_err(Error::Serial)?;
        }

        Ok(())
    }
}

fn calculate_crc(data: &[u8]) -> u16 {
    let mut crc = 0xFFFFu16;
    for d in data {
        crc ^= u16::from(*d);
        for _ in 0..u8::BITS {
            if (crc & 0x0001) != 0 {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    (crc & 0x00FF) << 8 | (crc & 0xFF00) >> 8
}

fn check_crc(data: &[u8]) -> bool {
    let target_length = data.len() - 2;
    let calculated_crc = calculate_crc(&data[..target_length]);

    calculated_crc.eq(&u16::from_be_bytes([
        data[data.len() - 2],
        data[data.len() - 1],
    ]))
}

#[cfg(test)]
mod tests {
    use crate::{calculate_crc, check_crc};

    #[test]
    fn test_crc_check() {
        const BYTES: [u8; 3] = [0x13, 0x12, 0x14];
        let crc = calculate_crc(&BYTES);
        let mut bytes_with_crc = [0u8; BYTES.len() + 2];
        bytes_with_crc[..BYTES.len()].copy_from_slice(&BYTES);
        bytes_with_crc[BYTES.len() + 0] = u8::try_from(crc >> u8::BITS).unwrap();
        bytes_with_crc[BYTES.len() + 1] = u8::try_from(crc & 0xFF).unwrap();

        assert!(check_crc(&bytes_with_crc))
    }

    #[test]
    fn test_crc_from_sensor() {
        const BYTES: [u8; 8] = [97, 6, 0, 52, 0, 1, 0, 100];
        assert!(check_crc(&BYTES))
    }
}
