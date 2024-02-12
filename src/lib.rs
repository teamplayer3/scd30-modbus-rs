#![cfg_attr(not(feature = "std"), no_std)]

use core::time::Duration;
use embedded_io_async::{Read, ReadExactError, Write};
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

#[allow(dead_code)]
enum Command {
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

enum RegFunction {
    ReadHolding = 0x3,
    #[allow(dead_code)]
    ReadInput = 0x4,
    WriteHolding = 0x6,
}

impl From<RegFunction> for u8 {
    fn from(f: RegFunction) -> Self {
        f as u8
    }
}

const RESPONSE_TIME: Duration = Duration::from_millis(5);
const MODBUS_ADDR: u8 = 0x61;

/// Measurements received by the sensor.
#[derive(Debug)]
pub struct Measurement {
    /// Co2 measuring [ppm].
    pub co2: f32,
    /// Humidity measuring [%].
    pub humidity: f32,
    /// Temperature measuring [°C].
    pub temperature: f32,
}

/// Scd30 driver.
pub struct Scd30<Serial> {
    serial: Serial,
}

/// See the [datasheet] for I²c parameters.
///
/// [datasheet]: https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/9.5_CO2/Sensirion_CO2_Sensors_SCD30_Interface_Description.pdf
impl<Serial> Scd30<Serial>
where
    Serial: Read + Write,
{
    pub const BAUDRATE: u16 = 19200;

    /// Returns an [Scd30] instance with the default address 0x61 shifted one place to the left.
    /// You may or may not need this bitshift depending on the byte size of
    /// your [I²c](embedded_hal::blocking::i2c) peripheral.
    pub fn new(serial: Serial) -> Self {
        Scd30 { serial }
    }

    /// Resets the sensor as it was after power on.
    ///
    /// Intern saved data persists and is loaded again.
    pub async fn soft_reset(&mut self) -> Result<(), Error<Serial::Error>> {
        self.serial.flush().await.map_err(Error::Serial)?;
        self.write(RegFunction::WriteHolding.into(), Command::SoftReset, 0x0001)
            .await?;
        if !self
            .check_resp_data::<8, 6>(RegFunction::WriteHolding.into(), 4, 0x0001)
            .await?
        {
            return Err(Error::GotWrongResponse);
        }
        Ok(())
    }

    /// Stops interval measuring.
    pub async fn stop_measuring(&mut self) -> Result<(), Error<Serial::Error>> {
        self.write(
            RegFunction::WriteHolding.into(),
            Command::StopContinuousMeasurement,
            0x001,
        )
        .await?;
        if !self
            .check_resp_data::<8, 6>(RegFunction::WriteHolding.into(), 4, 0x0001)
            .await?
        {
            return Err(Error::GotWrongResponse);
        }
        Ok(())
    }

    /// Enable or disable automatic self calibration (ASC).
    ///
    /// According to the datasheet, the sensor should be active continously for at least
    /// 7 days to find the initial parameter for ASC. The sensor has to be exposed to
    /// at least 1 hour of fresh air (~400ppm CO₂) per day.
    pub async fn set_automatic_calibration(
        &mut self,
        enable: bool,
    ) -> Result<(), Error<Serial::Error>> {
        let data = match enable {
            true => 0x0001,
            _ => 0,
        };
        self.write(
            RegFunction::ReadHolding.into(),
            Command::SetAutomaticSelfCalibration,
            data,
        )
        .await?;
        if !self
            .check_resp_data::<7, 5>(RegFunction::ReadHolding.into(), 3, data)
            .await?
        {
            return Err(Error::GotWrongResponse);
        }
        Ok(())
    }

    /// Set forced recalibration value (FCR).
    ///
    /// CO2 reference must be in unit ppm.
    pub async fn force_recalibrate_with_value(
        &mut self,
        reference: u16,
    ) -> Result<(), Error<Serial::Error>> {
        self.write(
            RegFunction::WriteHolding.into(),
            Command::ForcedRecalibrationValue,
            reference,
        )
        .await?;
        if !self
            .check_resp_data::<8, 6>(RegFunction::WriteHolding.into(), 4, reference)
            .await?
        {
            return Err(Error::GotWrongResponse);
        }
        Ok(())
    }

    /// Get the calibration reference which can be set by [`Scd30::force_recalibrate_with_value()`].
    pub async fn get_forced_recalibration_value(&mut self) -> Result<u16, Error<Serial::Error>> {
        self.write(
            RegFunction::ReadHolding.into(),
            Command::ForcedRecalibrationValue,
            0x0001,
        )
        .await?;
        Timer::after(RESPONSE_TIME).await;
        let resp = self.read_n::<7, 5>(RegFunction::ReadHolding.into()).await?;
        Ok(u16::from_be_bytes([resp[3], resp[4]]))
    }

    /// Set a temperature offset [°C] which will be applied to the received measures.
    ///
    /// The on-board RH/T sensor is influenced by thermal self-heating of SCD30 and other
    /// electrical components. Design-in alters the thermal properties of SCD30 such that
    /// temperature and humidity offsets may occur when operating the sensor in end-customer
    /// devices. Compensation of those effects is achievable by writing the temperature offset
    /// found in continuous operation of the device into the sensor.
    pub async fn set_temperature_offset(
        &mut self,
        offset: u16,
    ) -> Result<(), Error<Serial::Error>> {
        self.write(
            RegFunction::WriteHolding.into(),
            Command::SetTemperatureOffset,
            offset,
        )
        .await?;
        if !self
            .check_resp_data::<8, 6>(RegFunction::WriteHolding.into(), 4, offset)
            .await?
        {
            return Err(Error::GotWrongResponse);
        }
        Ok(())
    }

    /// Start measuring without mbar compensation.
    pub async fn start_measuring(&mut self) -> Result<(), Error<Serial::Error>> {
        self.start_measuring_with_mbar(0).await
    }

    pub async fn set_measurement_interval(
        &mut self,
        interval: Duration,
    ) -> Result<(), Error<Serial::Error>> {
        let secs = interval.as_secs();
        debug_assert!((2..=1800).contains(&secs));
        let secs = u16::try_from(secs).unwrap();
        self.write(
            RegFunction::WriteHolding.into(),
            Command::SetMeasurementInterval,
            secs,
        )
        .await?;
        if !self
            .check_resp_data::<8, 6>(RegFunction::WriteHolding.into(), 4, secs)
            .await?
        {
            return Err(Error::GotWrongResponse);
        }
        Ok(())
    }

    /// Start measuring with mbar (pressure) compensation.
    ///
    /// Starts continuous measurement of the SCD30 to measure CO2 concentration, humidity and temperature.
    /// Measurement data which is not read from the sensor will be overwritten. The measurement interval is
    /// adjustable via [`Scd30::set_measurement_interval()`], initial measurement rate is 2s.
    ///
    /// Continuous measurement status is saved in non-volatile memory. When the sensor is powered down
    /// while continuous measurement mode is active SCD30 will measure continuously after repowering
    /// without sending the measurement command.
    ///
    /// The CO2 measurement value can be compensated for ambient pressure by feeding the pressure value in
    /// mBar to the sensor. Setting the ambient pressure will overwrite previous settings of altitude
    /// compensation. Setting the argument to zero will deactivate the ambient pressure compensation
    /// (default ambient pressure = 1013.25 mBar). For setting a new ambient pressure when continuous
    /// measurement is running the whole command has to be written to SCD30.
    pub async fn start_measuring_with_mbar(
        &mut self,
        pressure: u16,
    ) -> Result<(), Error<Serial::Error>> {
        debug_assert!(pressure == 0 || (700..=1400).contains(&pressure));
        self.write(
            RegFunction::WriteHolding.into(),
            Command::StartContinuousMeasurement,
            pressure,
        )
        .await?;
        if !self
            .check_resp_data::<8, 6>(RegFunction::WriteHolding.into(), 4, pressure)
            .await?
        {
            return Err(Error::GotWrongResponse);
        }
        Ok(())
    }

    /// Returns if measures are ready.
    ///
    /// Is used to determine if a measurement can be read from the sensor’s buffer. Whenever
    /// there is a measurement available from the internal buffer it returns true.
    pub async fn data_ready(&mut self) -> Result<bool, Error<Serial::Error>> {
        self.write(
            RegFunction::ReadHolding.into(),
            Command::GetDataReadyStatus,
            0x0001,
        )
        .await?;
        self.check_resp_data::<7, 5>(RegFunction::ReadHolding.into(), 3, 0x0001)
            .await
    }

    /// Read measurements from the sensor.
    ///
    /// This function tests first if data is available as [`Scd30::data_ready()`] does. If data
    /// is ready it is returned as a `Some` else it returns `None`.
    pub async fn read(&mut self) -> Result<Option<Measurement>, Error<Serial::Error>> {
        match self.data_ready().await {
            Ok(true) => {
                self.write(
                    RegFunction::ReadHolding.into(),
                    Command::ReadMeasurement,
                    0x0006,
                )
                .await?;
                let buffer = self
                    .read_n::<17, 15>(RegFunction::ReadHolding.into())
                    .await?;
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

    async fn check_resp_data<const NB: usize, const N: usize>(
        &mut self,
        func: u8,
        high_byte_index: usize,
        evaluate_data: u16,
    ) -> Result<bool, Error<Serial::Error>> {
        Timer::after(RESPONSE_TIME).await;
        let resp = self.read_n::<NB, N>(func).await?;
        Ok(u16::from_be_bytes([resp[high_byte_index], resp[high_byte_index + 1]]) == evaluate_data)
    }

    async fn read_n<const NB: usize, const N: usize>(
        &mut self,
        func: u8,
    ) -> Result<[u8; N], Error<Serial::Error>> {
        let mut buffer = [0u8; NB];

        let mut byte_index = 0;
        loop {
            match byte_index {
                0 => {
                    let _skipped = self.skip_until_byte(MODBUS_ADDR).await?;
                    buffer[0] = MODBUS_ADDR;
                }
                1 => {
                    let byte = self.read_byte().await?;
                    if byte == func {
                        buffer[1] = func;
                    } else {
                        byte_index = 0;
                        continue;
                    }
                }
                i if i == NB => match check_crc(&buffer) {
                    true => break,
                    _ => return Err(Error::WrongCrc),
                },
                i => buffer[i] = self.read_byte().await?,
            }
            byte_index += 1;
        }

        let mut output = [0u8; N];
        output.copy_from_slice(&buffer[..N]);

        Ok(output)
    }

    async fn read_byte(&mut self) -> Result<u8, Error<Serial::Error>> {
        let mut buf = [0u8; 1];
        self.serial
            .read_exact(&mut buf)
            .await
            .map_err(|e| match e {
                ReadExactError::Other(e) => Error::Serial(e),
                ReadExactError::UnexpectedEof => panic!(),
            })?;

        Ok(buf[0])
    }

    async fn skip_until_byte(&mut self, byte: u8) -> Result<usize, Error<Serial::Error>> {
        let mut skipped = 0;
        loop {
            let read_byte = self.read_byte().await?;
            if read_byte == byte {
                break;
            }
            skipped += 1;
        }
        Ok(skipped)
    }

    async fn write(
        &mut self,
        func: u8,
        cmd: Command,
        data: u16,
    ) -> Result<(), Error<Serial::Error>> {
        let mut buffer = [0u8; 6 + 2];

        buffer[0] = MODBUS_ADDR;
        buffer[1] = func;

        buffer[2..=3].copy_from_slice(&(cmd as u16).to_be_bytes());
        buffer[4..=5].copy_from_slice(&data.to_be_bytes());

        let crc = calculate_crc(&buffer[..6]);
        buffer[6..=7].copy_from_slice(&crc.to_be_bytes());
        self.write_n(&buffer).await
    }

    async fn write_n(&mut self, data: &[u8]) -> Result<(), Error<Serial::Error>> {
        self.serial.write_all(data).await.map_err(Error::Serial)
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
