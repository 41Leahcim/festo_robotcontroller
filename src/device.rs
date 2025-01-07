//! This module contains everything related to the base `Device` type.
//! The `Device` type is responsible for communicating with the device.
//! It's also the base struct for most types of subdevices.

use crate::controller::Controller;
use core::fmt::{self, Debug, Formatter};
use ethercrab::error::Error as EthercrabError;

pub mod servo;

/// An error returned while resetting the device
pub enum ResetError {
    /// Failed to reset the device
    ResetFailed(usize),

    /// Failed to reset with a warning
    ResetFailedWarning(usize),

    /// Failed to reset with a fault
    ResetFailedFault(usize),

    /// There is an existing reference to the requisted device.
    /// When returned by the enable error, the device doesn't exist.
    DeviceInUse(usize, EthercrabError),
}

impl ResetError {
    /// # Returns
    /// The device returns the slave contained in the error message
    pub const fn device(&self) -> usize {
        match self {
            Self::ResetFailed(device)
            | Self::ResetFailedWarning(device)
            | Self::ResetFailedFault(device)
            | Self::DeviceInUse(device, _) => *device,
        }
    }
}

impl Debug for ResetError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::ResetFailed(device_number) => {
                write!(f, "Resetting device number {device_number} failed")
            }
            Self::ResetFailedFault(device_number) => write!(
                f,
                "Resetting device number {device_number} failed with only the fault bit being set"
            ),
            Self::ResetFailedWarning(device_number) => write!(
                f,
                "Resetting device number {device_number} failed with only the warning bit being set"
            ),
            Self::DeviceInUse(device, error) => {
                write!(f, "Resetting device {device} failed:\n{error:?}")
            }
        }
    }
}

/// An error returned while enabling (constructing) a device
pub enum EnableError {
    /// Failed to reset the device
    ResetFailed(ResetError),

    /// Device initialization failed because of a timeout
    Timeout(usize),

    /// Initialization failed with an unknown error
    Failed(usize),
}

impl Debug for EnableError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::ResetFailed(reset_error) => write!(
                f,
                "Enable drive {} after unsuccesful reset:\n{reset_error:?}",
                reset_error.device()
            ),
            Self::Timeout(device) => write!(f, "Timeout: Enable drive {device} unsuccessful"),
            Self::Failed(device) => write!(f, "Enable drive {device} unsuccessful"),
        }
    }
}

/// The operation mode byte value
#[derive(Debug, Clone, Copy)]
#[expect(dead_code)]
enum OperationMode {
    /// Not operating
    None,

    /// Change position mode
    ProfilePosition,

    /// Setting velocity mode
    Velocity,

    /// Change velocity mode
    ProfileVelocity,

    /// Torque mode
    Torque,

    /// Moving to home
    Homing = 6,

    /// Interpolated positioning mode
    InterpolatedPosition,

    /// Cylcing synchronization mode
    CyclingSyncPosition,

    /// Cycling synchronization velocity mode
    CyclingSyncVelocity,

    /// Cycling synchronization torque mode
    CyclingSyncTorque,

    /// Jogging in a specified direction
    Jog = 253,
}

/// An error happened while setting a new mode
pub struct SetModeError(usize, OperationMode);

impl Debug for SetModeError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        write!(f, "Failed to set device {} to mode {:?}", self.0, self.1)
    }
}

/// The error status of the device
#[derive(Debug, PartialEq, Eq)]
pub enum DeviceError {
    /// Fault bit is set on the device
    Fault,

    /// Warning bit is set on the device
    Warning,

    /// Both fault and warning bits are set on the device
    FaultAndWarning,

    /// The fault and warning bits aren't set on the device
    Ok,
}

/// Bits specifying the current control value
enum ControlBit {
    /// Switch the device on
    SwitchOn,

    /// Enable the drive voltage
    EnableVoltage,

    /// Quickly stop the device
    QuickStop,

    /// Enable device operation
    EnableOperation,

    /// Control bit
    Control4,

    /// Control bit
    Control5,

    /// Control bit
    Control6,

    /// Reset on fault
    FaultReset,

    /// Stop the device
    Halt,

    /// Control bit
    Control9,
}

/// Bits specifying the current status in the status word
#[expect(dead_code)]
enum StatusWordBit {
    /// Whether the device is ready to switch on
    ReadyToSwitchOn,

    /// Whether the device is switched on
    SwitchedOn,

    /// Whether device operation is currently enabled
    OperationEnabled,

    /// Whether a fault was detected by the device
    Fault,

    /// Whether the drive receives a voltage
    VoltageEnabled,

    /// Whether the device is in quick stop active state
    QuickStop,

    /// The drive can't switch on
    SwitchOnDisabled,

    /// The device detected a warning condition
    Warning,

    /// Manufacturer specific bit
    ManFsp,

    /// Remote control
    Remote,

    /// Requested motion has completed
    MotionComplete,

    /// Ack start or Ref reached
    AckStartRefReached = 12,

    /// Drive moved to home position
    DriveHomed = 15,
}

#[expect(dead_code)]
/// Mapped PDO bytes
enum MappedPdo {
    /// Control or status word
    ControlStatusWord,

    /// Mode of operation or mode of operation display
    OperationMode = 2,

    /// Target position of actual position
    Position = 3,

    /// Requested velocity
    ProfileVelocity = 7,

    /// Target velocity or actual velocity
    Velocity = 11,

    /// Target torque or actual torque
    Torque = 15,

    /// Velocity difference
    VelocityOffset = 17,

    /// Torque difference
    TorqueOffset = 21,
}

/// A timeout happened
pub struct Timeout;

/// A generic device type.
/// All kinds of subdevices should contain this struct.
pub struct Device<'device, 'controller: 'device, const MAX_DEVICES: usize, const PDI_LENGTH: usize>
{
    /// The device ID
    id: usize,

    /// The controller used
    controller: &'device Controller<'controller, MAX_DEVICES, PDI_LENGTH>,
}

impl<'device, 'controller: 'device, const MAX_DEVICES: usize, const PDI_LENGTH: usize>
    Device<'device, 'controller, MAX_DEVICES, PDI_LENGTH>
{
    /// Creates a new generic device.
    ///
    /// # Parameters
    /// `controller`: A reference to the controller struct to communicate with the device.
    /// `device_number`: The id of the device to communicate with
    ///
    /// # Errors
    /// Returns an error if:
    /// - The device failed to reset
    /// - The configuration timed out
    /// - Failed to configure device
    ///
    /// # Returns
    /// Returns the new generic device or error
    pub async fn new(
        controller: &'controller Controller<'_, MAX_DEVICES, PDI_LENGTH>,
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
        let mut timeout = 1_000_000;
        while !(result.get_bit(
            StatusWordBit::VoltageEnabled as u8,
            MappedPdo::ControlStatusWord,
        ) && result.get_bit(StatusWordBit::QuickStop as u8, MappedPdo::ControlStatusWord))
            && timeout > 0
        {
            result.set_bit(ControlBit::QuickStop as u8, MappedPdo::ControlStatusWord);
            result.set_bit(
                ControlBit::EnableVoltage as u8,
                MappedPdo::ControlStatusWord,
            );
            controller.cycle().await;
            timeout -= 1;
        }
        eprintln!("Enable voltage is on");
        while !(result.get_bit(
            StatusWordBit::OperationEnabled as u8,
            MappedPdo::ControlStatusWord,
        ) && result.get_bit(
            StatusWordBit::SwitchedOn as u8,
            MappedPdo::ControlStatusWord,
        )) && timeout > 0
        {
            timeout -= 1;
            result.set_bit(
                ControlBit::EnableOperation as u8,
                MappedPdo::ControlStatusWord,
            );
            result.set_bit(ControlBit::SwitchOn as u8, MappedPdo::ControlStatusWord);
            controller.cycle().await;
        }
        eprintln!("Device turned on");
        if result.get_bit(
            StatusWordBit::VoltageEnabled as u8,
            MappedPdo::ControlStatusWord,
        ) && result.get_bit(StatusWordBit::QuickStop as u8, MappedPdo::ControlStatusWord)
            && result.get_bit(
                StatusWordBit::OperationEnabled as u8,
                MappedPdo::ControlStatusWord,
            )
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

    /// Resets the device to it's original state.
    ///
    /// # Errors
    /// Returns an error if:
    /// - The subdevice couldn't be retrieved (already in use or doesn't exist)
    /// - Fault or warning bit was still set after multiple tries to reset the device
    ///
    /// # Returns
    /// `()` or error
    pub async fn reset(&mut self) -> Result<(), ResetError> {
        if self.controller.verbose() {
            log::info!("Resetting device number: {}", self.id);
        }
        self.controller
            .group()
            .subdevice(self.controller.main_device(), self.id)
            .map_err(|error| ResetError::DeviceInUse(self.id, error))?
            .outputs_raw_mut()
            .fill(0);

        if self.controller.verbose() {
            log::info!("Wait for empty frame device number: {}", self.id);
        }
        self.controller.cycle().await;

        let mut retries = 1000;
        while self.get_error() != DeviceError::Ok && retries > 0 {
            retries -= 1;
            self.set_bit(ControlBit::FaultReset as u8, MappedPdo::ControlStatusWord);
            if self.controller.verbose() {
                log::info!("Waiting on fault device number: {}", self.id);
            }
            self.controller.cycle().await;
            self.unset_bit(ControlBit::FaultReset as u8, MappedPdo::ControlStatusWord);
        }
        eprintln!("Device OK");

        match (
            self.get_bit(StatusWordBit::Fault as u8, MappedPdo::ControlStatusWord),
            &&self.get_bit(StatusWordBit::Warning as u8, MappedPdo::ControlStatusWord),
        ) {
            (false, false) => {
                if self.controller.verbose() {
                    log::info!("Ressetting device number: {} done", self.id);
                }
                Ok(())
            }
            (false, true) => Err(ResetError::ResetFailedWarning(self.id)),
            (true, false) => Err(ResetError::ResetFailedFault(self.id)),
            (true, true) => Err(ResetError::ResetFailed(self.id)),
        }
    }

    /// Sets the specified bit in the device.
    ///
    /// # Parameters
    /// `bit`: The bit to be set
    /// `byte`: The byte containing that bit
    ///
    /// # Returns
    /// The new value of the byte or 0
    #[expect(
        clippy::needless_pass_by_ref_mut,
        reason = "Mutable reference is used to decrease the chance of having multiple SubDevice refs at a time"
    )]
    fn set_bit(&mut self, mut bit: u8, byte: MappedPdo) -> u8 {
        let Ok(mut sub_device) = self
            .controller
            .group()
            .subdevice(self.controller.main_device(), self.id)
        else {
            return 0;
        };
        let byte = (byte as usize + usize::from(bit / 8)) % sub_device.outputs_raw().len();
        bit %= 8;
        let outputs = sub_device.outputs_raw_mut();
        outputs[byte] |= 1 << bit;
        outputs[byte]
    }

    /// Clears the specified bit in the device
    ///
    /// # Parameters
    /// `bit`: The bit to clear
    /// `byte`: The byte containing that bit
    ///
    /// # Returns
    /// The new value of that byte or 0
    #[expect(
        clippy::needless_pass_by_ref_mut,
        reason = "Mutable reference is used to decrease the chance of having multiple SubDevice refs at a time"
    )]
    fn unset_bit(&mut self, mut bit: u8, byte: MappedPdo) -> u8 {
        let Ok(mut sub_device) = self
            .controller
            .group()
            .subdevice(self.controller.main_device(), self.id)
        else {
            return 0;
        };
        let byte = (byte as usize + usize::from(bit / 8)) % sub_device.outputs_raw().len();
        bit %= 8;

        let outputs = sub_device.outputs_raw_mut();
        outputs[byte] &= !(1 << bit);
        outputs[byte]
    }

    #[expect(
        clippy::needless_pass_by_ref_mut,
        reason = "Mutable reference is used to decrease the chance of having multiple SubDevice refs at a time"
    )]
    /// Reads the specified bit from the device
    ///
    /// # Parameters
    /// `bit`: The bit to read
    /// `byte`: The byte containing that bit
    ///
    /// # Returns
    /// The value of the requested bit
    fn get_bit(&mut self, mut bit: u8, byte: MappedPdo) -> bool {
        let Ok(sub_device) = self
            .controller
            .group()
            .subdevice(self.controller.main_device(), self.id)
        else {
            return false;
        };
        let byte = (byte as usize + usize::from(bit / 8)) % sub_device.inputs_raw().len();
        bit %= 8;
        (sub_device.inputs_raw()[byte] & (1 << bit)) != 0
    }

    /// Reads whether the device is currently enabled for operation
    ///
    /// # Returns
    /// Whether the device is currently enabled for operation
    fn ready_state(&mut self) -> bool {
        self.get_bit(
            StatusWordBit::OperationEnabled as u8,
            MappedPdo::ControlStatusWord,
        )
    }

    /// Clears the control bits
    ///
    /// # Returns
    /// Returns the new value of the control bytes
    fn unset_control(&mut self) -> u16 {
        self.unset_bit(ControlBit::Control4 as u8, MappedPdo::ControlStatusWord);
        self.unset_bit(ControlBit::Control5 as u8, MappedPdo::ControlStatusWord);
        let byte0 = self.unset_bit(ControlBit::Control6 as u8, MappedPdo::ControlStatusWord);
        let byte1 = self.unset_bit(ControlBit::Control9 as u8, MappedPdo::ControlStatusWord);
        (u16::from(byte1) << 8) | u16::from(byte0)
    }

    /// Reads the device error flags
    ///
    /// # Returns
    /// Returns whether the device is in warning, fault, fault + warning, or ok
    pub fn get_error(&mut self) -> DeviceError {
        match (
            self.get_bit(StatusWordBit::Fault as u8, MappedPdo::ControlStatusWord),
            self.get_bit(StatusWordBit::Warning as u8, MappedPdo::ControlStatusWord),
        ) {
            (false, false) => DeviceError::Ok,
            (false, true) => DeviceError::Warning,
            (true, false) => DeviceError::Fault,
            (true, true) => DeviceError::FaultAndWarning,
        }
    }

    /// Reads 2 bytes from the device.
    /// The specified byte will be the low byte, the next byte will be the high byte.
    ///
    /// # Errors
    /// Returns an error if:
    /// - Slave doesn't exist
    /// - Another reference to the subdevice exists
    ///
    /// # Returns
    /// Returns the 2 read bytes or an error
    pub fn get_16(&mut self, byte: u8) -> Result<u16, EthercrabError> {
        let sub_device = self
            .controller
            .group()
            .subdevice(self.controller.main_device(), self.id)?;
        let inputs = sub_device.inputs_raw();
        let byte = usize::from(byte);
        Ok(u16::from(inputs[byte]) | (u16::from(inputs[byte + 1]) << 8))
    }

    /// Sets a the device to the specified operation mode.
    ///
    /// # Errors
    /// Returns an error if the mode couldn't be set
    ///
    /// # Returns
    /// `()` or error
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

    /// Disables the device after use.
    ///
    /// # Errors
    /// Returns an error if the device didn't get disabled.
    pub async fn disable(mut self) -> Result<(), Timeout> {
        self.unset_bit(
            ControlBit::EnableOperation as u8,
            MappedPdo::ControlStatusWord,
        );
        self.unset_bit(ControlBit::SwitchOn as u8, MappedPdo::ControlStatusWord);
        self.controller.cycle().await;
        self.unset_bit(ControlBit::QuickStop as u8, MappedPdo::ControlStatusWord);
        self.unset_bit(
            ControlBit::EnableVoltage as u8,
            MappedPdo::ControlStatusWord,
        );

        let mut timeout = 1_000;
        while self.get_bit(
            StatusWordBit::OperationEnabled as u8,
            MappedPdo::ControlStatusWord,
        ) && timeout > 0
        {
            timeout -= 1;
            self.controller.cycle().await;
        }
        if timeout == 0 {
            Err(Timeout)
        } else {
            Ok(())
        }
    }
}
