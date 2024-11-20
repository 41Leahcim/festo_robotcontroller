use crate::controller::Controller;
use ethercrab::error::Error as EthercrabError;
use std::fmt::Debug;

pub mod servo;

pub enum ResetError {
    ResetFailed(usize),
    ResetFailedWithoutWarning(usize),
    ResetFailedWithoutFault(usize),
    DeviceNotFound(usize, EthercrabError),
}

impl ResetError {
    pub const fn device(&self) -> usize {
        match self {
            ResetError::ResetFailed(device)
            | ResetError::ResetFailedWithoutWarning(device)
            | ResetError::ResetFailedWithoutFault(device)
            | ResetError::DeviceNotFound(device, _) => *device,
        }
    }
}

impl Debug for ResetError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ResetFailed(device_number) => {
                write!(f, "Resetting device number {device_number} failed")
            }
            Self::ResetFailedWithoutWarning(device_number) => write!(
                f,
                "Resetting device number {device_number} failed without warning bit being set"
            ),
            Self::ResetFailedWithoutFault(device_number) => write!(
                f,
                "Resetting device number {device_number} failed without fault bit being set"
            ),
            Self::DeviceNotFound(device, error) => {
                write!(f, "Resetting device {device} failed:\n{error:?}")
            }
        }
    }
}

pub enum EnableError {
    ResetFailed(ResetError),
    Timeout(usize),
    Failed(usize),
}

impl Debug for EnableError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EnableError::ResetFailed(reset_error) => write!(
                f,
                "Enable drive {} after unsuccesful reset:\n{reset_error:?}",
                reset_error.device()
            ),
            EnableError::Timeout(device) => {
                write!(f, "Timeout: Enable drive {device} unsuccessful")
            }
            EnableError::Failed(device) => write!(f, "Enable drive {device} unsuccessful"),
        }
    }
}

#[derive(Debug, Clone, Copy)]
#[expect(dead_code)]
enum OperationMode {
    None,
    ProfilePosition,
    Velocity,
    ProfileVelocity,
    Torque,

    Homing = 6,
    InterpolatedPosition,
    CyclingSyncPosition,
    CyclingSyncVelocity,
    CyclingSyncTorque,

    Jog = 253,
}

pub struct SetModeError(usize, OperationMode);

impl Debug for SetModeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Failed to set device {} to mode {:?}", self.0, self.1)
    }
}

/// The error status of the device
#[derive(Debug, PartialEq, Eq)]
pub enum DeviceError {
    Fault,
    Warning,
    FaultAndWarning,
    Ok,
}

enum ControlBit {
    SwitchOn,
    EnableVoltage,
    QuickStop,
    EnableOperation,
    Control4,
    Control5,
    Control6,
    FaultReset,
    Halt,
    Control9,
}

#[expect(dead_code)]
enum StatusWordBit {
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    Fault,
    VoltageEnabled,
    QuickStop,
    SwitchOnDisabled,
    Warning,
    ManFsp,
    Remote,
    MotionComplete,
    /// Ack start or Ref reached
    AckStartRefReached = 12,
    DriveHomed = 15,
}

#[expect(dead_code)]
enum MappedPdo {
    /// Control or status word
    ControlStatusWord,

    /// Mode of operation or mode of operation display
    OperationMode = 2,

    /// Target position of actual position
    Position = 3,
    ProfileVelocity = 7,

    /// Target velocity or actual velocity
    Velocity = 11,

    /// Target torque or actual torque
    Torque = 15,

    VelocityOffset = 17,
    TorqueOffset = 21,
}

pub struct Device<'device, 'controller: 'device> {
    id: usize,
    controller: &'device Controller<'controller>,
}

impl<'device, 'controller: 'device> Device<'device, 'controller> {
    pub async fn new(
        controller: &'controller Controller<'_>,
        device_number: usize,
    ) -> Result<Self, EnableError> {
        if controller.verbose() {
            log::info!("Start enabling drive {device_number}");
        }
        let mut result = Self {
            id: device_number,
            controller,
        };
        result.reset().await.map_err(EnableError::ResetFailed)?;
        while !result.get_bit(ControlBit::QuickStop as u8, 0) {
            result.set_bit(ControlBit::QuickStop as u8, 0);
            result.set_bit(ControlBit::EnableVoltage as u8, 0);
            controller.cycle().await;
        }
        let mut timeout = 1_000_000;
        while !result.get_bit(StatusWordBit::OperationEnabled as u8, 0) && timeout > 0 {
            timeout -= 1;
            result.set_bit(ControlBit::EnableOperation as u8, 0);
            result.set_bit(ControlBit::SwitchOn as u8, 0);
            controller.cycle().await;
        }
        if result.get_bit(StatusWordBit::VoltageEnabled as u8, 0)
            && result.get_bit(StatusWordBit::QuickStop as u8, 0)
            && result.get_bit(StatusWordBit::OperationEnabled as u8, 0)
        {
            if controller.verbose() {
                log::info!("Enable drive {device_number} successful");
            }
            Ok(result)
        } else if timeout == 0 {
            Err(EnableError::Timeout(device_number))
        } else {
            Err(EnableError::Failed(device_number))
        }
    }

    pub async fn reset(&mut self) -> Result<(), ResetError> {
        if self.controller.verbose() {
            log::info!("Resetting device number: {}", self.id);
        }
        {
            let mut sub_device = self
                .controller
                .group()
                .subdevice(self.controller.main_device(), self.id)
                .map_err(|error| ResetError::DeviceNotFound(self.id, error))?;
            sub_device.outputs_raw_mut().fill(0);
        }

        if self.controller.verbose() {
            log::info!("Wait for empty frame device number: {}", self.id);
        }
        self.controller.cycle().await;

        let mut retries = 1000;
        while self.get_error() != DeviceError::Ok && retries > 0 {
            retries -= 1;
            self.set_bit(ControlBit::FaultReset as u8, 0);
            if self.controller.verbose() {
                log::info!("Waiting on fault device number: {}", self.id);
            }
            self.controller.cycle().await;
            self.unset_bit(ControlBit::FaultReset as u8, 0);
        }

        match (
            self.get_bit(StatusWordBit::Fault as u8, 0),
            &&self.get_bit(StatusWordBit::Warning as u8, 0),
        ) {
            (false, false) => {
                if self.controller.verbose() {
                    log::info!("Ressetting device number: {} done", self.id);
                }
                Ok(())
            }
            (false, true) => Err(ResetError::ResetFailedWithoutFault(self.id)),
            (true, false) => Err(ResetError::ResetFailedWithoutWarning(self.id)),
            (true, true) => Err(ResetError::ResetFailed(self.id)),
        }
    }

    fn set_bit(&mut self, mut bit: u8, byte: u8) -> u8 {
        let Ok(mut sub_device) = self
            .controller
            .group()
            .subdevice(self.controller.main_device(), self.id)
        else {
            return 0;
        };
        let byte = (usize::from(byte) + usize::from(bit / 8)) % sub_device.outputs_raw().len();
        bit %= 8;
        let outputs = sub_device.outputs_raw_mut();
        outputs[byte] |= 1 << bit;
        outputs[byte]
    }

    fn unset_bit(&mut self, mut bit: u8, byte: u8) -> u8 {
        let Ok(mut sub_device) = self
            .controller
            .group()
            .subdevice(self.controller.main_device(), self.id)
        else {
            return 0;
        };
        let byte = (usize::from(byte) + usize::from(bit / 8)) % sub_device.outputs_raw().len();
        bit %= 8;

        let outputs = sub_device.outputs_raw_mut();
        outputs[byte] &= !(1 << bit);
        outputs[byte]
    }

    fn get_bit(&mut self, mut bit: u8, byte: u8) -> bool {
        let Ok(sub_device) = self
            .controller
            .group()
            .subdevice(self.controller.main_device(), self.id)
        else {
            return false;
        };
        let byte = (usize::from(byte) + usize::from(bit / 8)) % sub_device.inputs_raw().len();
        bit %= 8;
        (sub_device.inputs_raw()[byte] & (1 << bit)) != 0
    }

    fn ready_state(&mut self) -> bool {
        self.get_bit(StatusWordBit::OperationEnabled as u8, 0)
    }

    fn unset_control(&mut self) -> u16 {
        self.unset_bit(
            ControlBit::Control4 as u8,
            MappedPdo::ControlStatusWord as u8,
        );
        self.unset_bit(
            ControlBit::Control5 as u8,
            MappedPdo::ControlStatusWord as u8,
        );
        let byte0 = self.unset_bit(
            ControlBit::Control6 as u8,
            MappedPdo::ControlStatusWord as u8,
        );
        let byte1 = self.unset_bit(
            ControlBit::Control9 as u8,
            MappedPdo::ControlStatusWord as u8,
        );
        u16::from(byte1) << 8 | u16::from(byte0)
    }

    pub fn get_error(&mut self) -> DeviceError {
        match (
            self.get_bit(StatusWordBit::Fault as u8, 0),
            self.get_bit(StatusWordBit::Warning as u8, 0),
        ) {
            (false, false) => DeviceError::Ok,
            (false, true) => DeviceError::Warning,
            (true, false) => DeviceError::Fault,
            (true, true) => DeviceError::FaultAndWarning,
        }
    }

    pub fn get_16(&mut self, byte: u8) -> Result<u16, EthercrabError> {
        let sub_device = self
            .controller
            .group()
            .subdevice(self.controller.main_device(), self.id)?;
        let inputs = sub_device.inputs_raw();
        let byte = usize::from(byte);
        Ok(u16::from(inputs[byte]) | (u16::from(inputs[byte + 1]) << 8))
    }

    async fn set_mode(&mut self, mode: OperationMode) -> Result<(), SetModeError> {
        let mut timeout = 100;

        // Wait for mode to get active
        while timeout > 0
            && self
                .controller
                .group()
                .subdevice(self.controller.main_device(), self.id)
                .is_ok_and(|sub_device| {
                    sub_device.inputs_raw()[MappedPdo::OperationMode as usize] == mode as u8
                })
        {
            timeout -= 1;
            self.unset_control();
            if let Ok(mut sub_device) = self
                .controller
                .group()
                .subdevice(self.controller.main_device(), self.id)
            {
                sub_device.outputs_raw_mut()[MappedPdo::OperationMode as usize] = mode as u8;
            }
            self.controller.cycle().await;
        }
        if self
            .controller
            .group()
            .subdevice(self.controller.main_device(), self.id)
            .is_ok_and(|sub_device| {
                sub_device.inputs_raw()[MappedPdo::OperationMode as usize] != mode as u8
            })
        {
            return Err(SetModeError(self.id, mode));
        }
        self.unset_control();
        if self.controller.verbose() {
            log::info!("Arrived in mode {mode:?}");
        }
        Ok(())
    }
}
