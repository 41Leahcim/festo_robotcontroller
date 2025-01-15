//! This module contains everything related to the `Servo` drive struct.
//! The `Servo` drive struct can control Servo's controlled by most Festo Servomotor drives.

use super::{Device, EnableError, OperationMode, SetModeError, StatusWordBit, Timeout};
use crate::{
    controller::Controller,
    device::{ControlBit, MappedPdo},
};
use core::fmt::{self, Debug, Formatter};
use ethercrab::error::Error as EthercrabError;

/// An error returned while moving the servo to it's default (home) position
pub enum HomingError {
    /// The drive is disabled
    DeviceDisabled(usize),

    /// The mode couldn't be set to the requested value
    SetMode(SetModeError),
}

impl Debug for HomingError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::DeviceDisabled(device) => {
                write!(f, "Homing not possible, device {device} is disabled")
            }
            Self::SetMode(error) => write!(f, "Error while setting homing mode: {error:?}"),
        }
    }
}

/// An error returned while slowly moving the servo in the requested direction
pub enum JoggingError {
    /// The device is disabled
    DeviceDisabled(usize),

    /// The device couldn't be set to the requested mode
    SetMode(SetModeError),
}

impl Debug for JoggingError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::DeviceDisabled(device) => {
                write!(f, "Jogging not possible device {device} is disabled")
            }
            Self::SetMode(error) => write!(f, "Error while setting jogging mode: {error:?}"),
        }
    }
}

/// Something went wrong while trying to move the servo to a specific position,
/// possibly with a specific velocity.
pub enum MovementError {
    /// The drive is disabled
    DriveDisabled(usize),

    /// Communication failed or there was an existing reference to the device.
    Ethercat(EthercrabError),

    /// Failed to set the device to the requested mode
    SetMode(SetModeError),
}

impl Debug for MovementError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::DriveDisabled(device) => {
                write!(f, "Drive {device} is disabled, movement not possible")
            }
            Self::Ethercat(error) => write!(f, "{error}"),
            Self::SetMode(error) => write!(f, "Error while setting movement mode: {error:?}"),
        }
    }
}

/// An error returned while trying to move the servo with full control
pub enum FullControlMovementError {
    /// Failed to set the acceleration to the requested value
    WritingAccelerationFailed(usize, EthercrabError),

    /// Failed to set the deceleration to the requested value
    WritingDecelerationFailed(usize, EthercrabError),

    /// Failed to move the servo to the requested position
    MovementFailed(MovementError),

    /// There's an existing reference to the device
    DeviceInUse(EthercrabError),
}

impl Debug for FullControlMovementError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::WritingAccelerationFailed(device, error) => {
                write!(
                    f,
                    "Writing acceleration failed on device {device}:\n{error:?}"
                )
            }
            Self::WritingDecelerationFailed(device, error) => {
                write!(
                    f,
                    "Writing deceleration failed on device {device}:\n{error:?}"
                )
            }
            Self::MovementFailed(move_drive_not_enabled) => {
                write!(f, "{move_drive_not_enabled:?}")
            }
            Self::DeviceInUse(error) => write!(f, "Device in use: {error:?}"),
        }
    }
}

/// How to calculate the new position for the servo
#[derive(Debug, PartialEq, Eq)]
pub enum MovementMode {
    /// Set the new position based on the current position
    Relative,

    /// Set the new position to the requested value
    Absolute,
}

/// The direction the device should jog in
enum JoggingDirection {
    /// Positive jogging direction
    Positive,

    /// Negative jogging direction
    Negative,
}

/// The struct responsible for controlling the servo motor.
pub struct Servo<'device, 'controller: 'device, const MAX_DEVICES: usize, const PDI_LENGTH: usize>(
    Device<'device, 'controller, MAX_DEVICES, PDI_LENGTH>,
);

impl<'device, 'controller: 'device, const MAX_DEVICES: usize, const PDI_LENGTH: usize>
    Servo<'device, 'controller, MAX_DEVICES, PDI_LENGTH>
{
    /// Creates a new device that can be used to control a servomotor.
    ///
    /// # Errors
    /// Returns an error if:
    /// - The device failed to reset
    /// - The configuration timed out
    /// - Failed to configure the device
    ///
    /// # Returns
    /// The new device or an error
    pub async fn new(
        controller: &'controller Controller<'_, MAX_DEVICES, PDI_LENGTH>,
        device_number: usize,
    ) -> Result<Self, EnableError> {
        Ok(Self(Device::new(controller, device_number).await?))
    }

    /// Returns a reference to the inner device for more specific control
    pub const fn device(&self) -> &Device<'device, 'controller, MAX_DEVICES, PDI_LENGTH> {
        &self.0
    }

    /// Returns a mutable reference to the inner device for more specific control
    pub fn device_mut(&mut self) -> &mut Device<'device, 'controller, MAX_DEVICES, PDI_LENGTH> {
        &mut self.0
    }

    /// Retrieves the current position of the servo.
    ///
    ///  # Errors
    /// Returns an error if another reference to the device exists
    pub fn get_position(&mut self) -> Result<u32, EthercrabError> {
        /// The address of the actual current position
        const POSITION_ACTUAL_VALUE_ADDRESS: usize = 3;

        // Select the device
        let sub_device = self
            .0
            .controller
            .group()
            .subdevice(self.0.controller.main_device(), self.0.id)?;

        // Read the position as an u32
        Ok(sub_device
            .inputs_raw()
            .iter()
            .copied()
            .skip(POSITION_ACTUAL_VALUE_ADDRESS)
            .take(4)
            .enumerate()
            .map(|(i, value)| u32::from(value) << (8 * i))
            .sum())
    }

    /// Moves the servo to home (default position).
    ///
    /// # Errors
    /// Returns an error if:
    /// - The device is disabled
    /// - The servo can't be set to homing mode
    pub async fn home(&mut self, always: bool) -> Result<(), HomingError> {
        if !self.0.ready_state() {
            return Err(HomingError::DeviceDisabled(self.0.id));
        }

        // Set the device to the homing mode
        self.0
            .set_mode(OperationMode::Homing)
            .await
            .map_err(HomingError::SetMode)?;

        // Display a warning, if the device is already homed and the device shouldn't always home.
        if self.0.get_bit(
            StatusWordBit::DriveHomed as u8,
            MappedPdo::ControlStatusWord,
        ) && !always
        {
            log::info!("device already homed");
        } else {
            log::info!("device {} starting homing", self.0.id);
            // Clear the control bits, but set control bit 4
            self.0.unset_control();
            self.0
                .set_bit(ControlBit::Control4 as u8, MappedPdo::ControlStatusWord);

            // Wait until the device is homed
            while !self.0.get_bit(
                StatusWordBit::AckStartRefReached as u8,
                MappedPdo::ControlStatusWord,
            ) {
                self.0.controller.cycle().await;
            }

            // Clear the bit
            self.0
                .unset_bit(ControlBit::Control4 as u8, MappedPdo::ControlStatusWord);
        }
        Ok(())
    }

    /// Move the servo in the requested direction.
    ///
    /// # Errors
    /// Returns an error if:
    /// - The device is disabled
    /// - The servo couldn't be set to jogging mode
    async fn jog(&mut self, direction: JoggingDirection) -> Result<(), JoggingError> {
        if !self.0.ready_state() {
            return Err(JoggingError::DeviceDisabled(self.0.id));
        }

        // Set the jogging mode
        self.0
            .set_mode(OperationMode::Jog)
            .await
            .map_err(JoggingError::SetMode)?;

        // Clear the control bits
        self.0.unset_control();

        // Wait until the previous motion has completed
        while !self.0.get_bit(
            StatusWordBit::MotionComplete as u8,
            MappedPdo::ControlStatusWord,
        ) {
            self.0.controller.cycle().await;
        }

        // Set the jogging direction
        self.0.set_bit(
            match direction {
                JoggingDirection::Positive => ControlBit::Control4,
                JoggingDirection::Negative => ControlBit::Control5,
            } as u8,
            MappedPdo::ControlStatusWord,
        );
        Ok(())
    }

    /// Moves the servo in positive direction.
    ///
    /// # Errors
    /// Returns an error if:
    /// - The device is disabled
    /// - The servo couldn't be set to jogging mode
    pub async fn jog_positive(&mut self) -> Result<(), JoggingError> {
        if self.0.controller.verbose() {
            log::info!("Begin jog in positive direction");
        }
        self.jog(JoggingDirection::Positive).await
    }

    /// Moves the servo in negative direction.
    ///
    /// # Errors
    /// Returns an error if:
    /// - The device is disabled
    /// - The servo couldn't be set to jogging mode
    pub async fn jog_negative(&mut self) -> Result<(), JoggingError> {
        if self.0.controller.verbose() {
            log::info!("Begin jog in negative direction");
        }
        self.jog(JoggingDirection::Negative).await
    }

    /// Stop moving the servo (required position has been reached)
    pub async fn jog_stop(&mut self) {
        if self.0.controller.verbose() {
            log::info!("Stopping jog movement");
        }
        // Return if the device isn't operational
        if !self.0.ready_state() {
            return;
        }

        // Clear the control bits
        self.0.unset_control();

        // Wait until the motion is complete
        while !self.0.get_bit(
            StatusWordBit::MotionComplete as u8,
            MappedPdo::ControlStatusWord,
        ) {
            self.0.controller.cycle().await;
        }
    }

    /// Move the servo to the requested position.
    ///
    /// Precondition for positioning mode.
    /// The following conditions must be fulfilled for positioning mode:
    /// - Modes of operation display (0x6061) = 1
    /// - Statusword (0x6041) = 1X0X X11X X011 0111b
    ///     Control and monitoring
    ///     Object 0x6040: Controlword
    ///     The object controls the following functions of positioning mode:
    /// - bit 4: start motion command (new set - point)
    /// - Bit 5: Accept change immediately (change set immediately)
    ///     Motion control
    ///     Festo - CMMT - AS - SW - 2019 - 08C 303
    /// - Bit 6: Positioning type (absolute/relative)
    /// - Bit 8: Stop motion command (Halt)
    ///
    /// # Errors
    /// Returns an error if:
    /// - The drive is not enabled
    /// - The positition profile couldn't be set to set mode
    /// - The position couldn't be set
    pub async fn move_position(
        &mut self,
        target: i32,
        movement: MovementMode,
    ) -> Result<(), MovementError> {
        if self.0.controller.verbose() {
            log::info!(
                "Starting {movement:?} movement to position {target} of device {}",
                self.0.id
            );
        }
        if !self.0.ready_state() {
            return Err(MovementError::DriveDisabled(self.0.id));
        }
        // Set the direction to move in
        self.0
            .set_mode(OperationMode::ProfilePosition)
            .await
            .map_err(MovementError::SetMode)?;

        // Clear the control bits
        self.0.unset_control();

        // Set control bit 6 if the motion has to be relative to the current positon
        if movement == MovementMode::Relative {
            self.0
                .set_bit(ControlBit::Control6 as u8, MappedPdo::ControlStatusWord);
        }

        // Set the position to the requested value
        self.set_position(target, 3)
            .map_err(MovementError::Ethercat)?;

        // Perform an update cycle
        self.0.controller.cycle().await;

        // Clear the halt bit
        self.0
            .unset_bit(ControlBit::Halt as u8, MappedPdo::ControlStatusWord);

        // Set control bit 4
        self.0
            .set_bit(ControlBit::Control4 as u8, MappedPdo::ControlStatusWord);

        eprintln!("Waiting for ACK start");
        // Wait until the requested position has been reached
        while !self.0.get_bit(
            StatusWordBit::AckStartRefReached as u8,
            MappedPdo::ControlStatusWord,
        ) {
            self.0.controller.cycle().await;
        }

        eprintln!("Waiting until motion complete");
        // Wait until the motion is complete
        while !self.0.get_bit(
            StatusWordBit::MotionComplete as u8,
            MappedPdo::ControlStatusWord,
        ) {
            if self.0.controller.verbose() {
                let id = self.0.id;
                log::info!(
                    "Move device {id} {movement:?} : {target} {}",
                    self.get_position().map_err(MovementError::Ethercat)?
                );
            }

            //  Clear the control bits
            self.0.unset_control();
            self.0.controller.cycle().await;
        }
        if self.0.controller.verbose() {
            log::info!("Movement completed");
        }
        Ok(())
    }

    /// Move the servo to the requested position with the requested velocity.
    ///
    /// # Errors
    /// Returns an error if:
    /// - Another reference to the device already exists
    /// - The device isn't enabled
    /// - The position profile couldn't be set to set mode
    /// - The position couldn't be set
    pub async fn move_position_velocity(
        &mut self,
        target: i32,
        velocity: u32,
        movement: MovementMode,
    ) -> Result<(), MovementError> {
        // Set the profile velocity
        self.set_profile_velocity(velocity, MappedPdo::ProfileVelocity)
            .map_err(MovementError::Ethercat)?;

        // Move to the requested position
        self.move_position(target, movement).await
    }

    /// Move the servo at the requested velocity, acceleration, and deceleration to the
    /// requirested position.
    ///
    /// # Errors
    /// Returns an error if:
    /// - The device couldn't be found
    /// - The acceleration couldn't be set
    /// - The deceleration couldn't be set
    /// - Another reference to the device already exists
    /// - The device isn't enabled
    /// - The position mode couldn't be set to set mode
    /// - The position couldn't be set
    pub async fn move_position_velocity_acceleration(
        &mut self,
        target: i32,
        velocity: u32,
        acceleration: u32,
        deceleration: u32,
        movement: MovementMode,
    ) -> Result<(), FullControlMovementError> {
        {
            // Select the requested device
            let sub_device = self
                .0
                .controller
                .group()
                .subdevice(self.0.controller.main_device(), self.0.id)
                .map_err(FullControlMovementError::DeviceInUse)?;

            // Set the requested acceleration
            sub_device
                .sdo_write(0x6083, 0, acceleration)
                .await
                .map_err(|error| {
                    FullControlMovementError::WritingAccelerationFailed(self.0.id, error)
                })?;

            // Set the requested deceleration
            sub_device
                .sdo_write(0x6084, 0, deceleration)
                .await
                .map_err(|error| {
                    FullControlMovementError::WritingDecelerationFailed(self.0.id, error)
                })?;
        }

        // Move to the requested position with the requested velocity
        self.move_position_velocity(target, velocity, movement)
            .await
            .map_err(FullControlMovementError::MovementFailed)
    }

    #[expect(
        clippy::needless_pass_by_ref_mut,
        reason = "Mutable reference is used to decrease the chance of multiple references existing for the same device"
    )]
    /// Set the servo position to the requested value.
    fn set_position(&mut self, target: i32, byte: u8) -> Result<(), EthercrabError> {
        // Select the device
        let mut sub_device = self
            .0
            .controller
            .group()
            .subdevice(self.0.controller.main_device(), self.0.id)?;

        // Set the positon bit to the requested value
        let byte = usize::from(byte);
        sub_device.outputs_raw_mut()[byte..byte + size_of::<i32>()]
            .copy_from_slice(&target.to_le_bytes());
        Ok(())
    }

    /// Set the velocity to the requested value.
    ///
    /// # Errors
    /// Returns an error if another reference to the device exists
    #[expect(
        clippy::needless_pass_by_ref_mut,
        reason = "Mutable reference is used to decrease the chance of multiple references existing for the same device"
    )]
    fn set_profile_velocity(
        &mut self,
        velocity: u32,
        byte: MappedPdo,
    ) -> Result<(), EthercrabError> {
        // Select the device
        let mut sub_device = self
            .0
            .controller
            .group()
            .subdevice(self.0.controller.main_device(), self.0.id)?;

        // Set the requested profile velocity
        let byte = byte as usize;
        sub_device.outputs_raw_mut()[byte..byte + size_of::<u32>()]
            .copy_from_slice(&velocity.to_le_bytes());
        Ok(())
    }

    /// Disables the device after use.
    ///
    /// # Errors
    /// Returns an error if the device didn't get disabled.
    pub async fn disable(self) -> Result<(), Timeout> {
        self.0.disable().await
    }
}
