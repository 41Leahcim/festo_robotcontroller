use super::{Device, EnableError, OperationMode, SetModeError, StatusWordBit};
use crate::{controller::Controller, device::ControlBit};
use ethercrab::error::Error as EthercrabError;
use std::fmt::Debug;

#[derive(Debug)]
pub enum StartupError {
    FailedToInitialize,
    NoDevicesFound,
}

pub enum HomingError {
    DeviceDisabled(usize),
    SetMode(SetModeError),
}

impl Debug for HomingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DeviceDisabled(device) => {
                write!(f, "Homing not possible, device {device} not enabled")
            }
            Self::SetMode(error) => write!(f, "{error:?}"),
        }
    }
}

pub enum JoggingError {
    DeviceNotEnabled(usize),
    SetMode(SetModeError),
}

impl Debug for JoggingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DeviceNotEnabled(device) => {
                write!(f, "Jogging not possible device {device} not enabled")
            }
            Self::SetMode(error) => write!(f, "{error:?}"),
        }
    }
}

pub enum MovementError {
    DriveNotEnabled(usize),
    Ethercat(EthercrabError),
    SetMode(SetModeError),
}

impl Debug for MovementError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::DriveNotEnabled(device) => {
                write!(f, "Drive {device} not enabled, movement not possible")
            }
            Self::Ethercat(error) => write!(f, "{error}"),
            Self::SetMode(error) => write!(f, "{error:?}"),
        }
    }
}

pub enum FullControlMovementError {
    WritingAccelerationFailed(usize, EthercrabError),
    WritingDecelerationFailed(usize, EthercrabError),
    MovementFailed(MovementError),
    DeviceNotFound(EthercrabError),
}

impl Debug for FullControlMovementError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            FullControlMovementError::WritingAccelerationFailed(device, error) => {
                write!(
                    f,
                    "Writing acceleration failed on device {device}:\n{error:?}"
                )
            }
            FullControlMovementError::WritingDecelerationFailed(device, error) => {
                write!(
                    f,
                    "Writing deceleration failed on device {device}:\n{error:?}"
                )
            }
            FullControlMovementError::MovementFailed(move_drive_not_enabled) => {
                write!(f, "{move_drive_not_enabled:?}")
            }
            Self::DeviceNotFound(error) => write!(f, "{error:?}"),
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum MovementMode {
    Relative,
    Absolute,
}

enum JoggingDirection {
    Positive,
    Negative,
}

pub struct Servo<'device, 'controller: 'device>(Device<'device, 'controller>);

impl<'device, 'controller: 'device> Servo<'device, 'controller> {
    pub async fn new(
        controller: &'controller Controller<'_>,
        device_number: usize,
    ) -> Result<Self, EnableError> {
        Ok(Self(Device::new(controller, device_number).await?))
    }

    pub fn get_position(&mut self) -> Result<u32, EthercrabError> {
        const POSITION_ACTUAL_VALUE_ADDRESS: usize = 3;
        let sub_device = self
            .0
            .controller
            .group()
            .subdevice(self.0.controller.main_device(), self.0.id)?;
        let inputs = sub_device.inputs_raw();
        Ok((0..4)
            .map(|i| u32::from(inputs[POSITION_ACTUAL_VALUE_ADDRESS + i]) << (8 * i))
            .sum())
    }

    pub async fn home(&mut self, always: bool) -> Result<(), HomingError> {
        if !self.0.ready_state() {
            return Err(HomingError::DeviceDisabled(self.0.id));
        }
        self.0
            .set_mode(OperationMode::Homing)
            .await
            .map_err(HomingError::SetMode)?;
        if self.0.get_bit(StatusWordBit::DriveHomed as u8, 0) && !always {
            println!("device already homed");
        } else {
            println!("device {} starting homing", self.0.id);
            self.0.unset_control();
            self.0.set_bit(ControlBit::Control4 as u8, 0);
            while !self.0.get_bit(StatusWordBit::AckStartRefReached as u8, 0) {
                self.0.controller.cycle().await;
            }
            self.0.unset_bit(ControlBit::Control4 as u8, 0);
        }
        Ok(())
    }

    async fn jog(&mut self, direction: JoggingDirection) -> Result<(), JoggingError> {
        if !self.0.ready_state() {
            return Err(JoggingError::DeviceNotEnabled(self.0.id));
        }
        self.0
            .set_mode(OperationMode::Jog)
            .await
            .map_err(JoggingError::SetMode)?;
        self.0.unset_control();
        while !self.0.get_bit(StatusWordBit::MotionComplete as u8, 0) {
            self.0.controller.cycle().await;
        }
        self.0.set_bit(
            match direction {
                JoggingDirection::Positive => ControlBit::Control4,
                JoggingDirection::Negative => ControlBit::Control5,
            } as u8,
            0,
        );
        Ok(())
    }

    pub async fn jog_positive(&mut self) -> Result<(), JoggingError> {
        if self.0.controller.verbose() {
            println!("Begin jog in positive direction");
        }
        self.jog(JoggingDirection::Positive).await
    }

    pub async fn jog_negative(&mut self) -> Result<(), JoggingError> {
        if self.0.controller.verbose() {
            println!("Begin jog in negative direction");
        }
        self.jog(JoggingDirection::Negative).await
    }

    pub async fn jog_stop(&mut self) {
        if self.0.controller.verbose() {
            println!("Stopping jog movement");
        }
        if !self.0.ready_state() {
            return;
        }
        self.0.unset_control();
        while !self.0.get_bit(StatusWordBit::MotionComplete as u8, 0) {
            self.0.controller.cycle().await;
        }
    }

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
    pub async fn move_position(
        &mut self,
        target: i32,
        movement: MovementMode,
    ) -> Result<(), MovementError> {
        if self.0.controller.verbose() {
            println!(
                "Starting {movement:?} movement to position {target} of device {}",
                self.0.id
            );
        }
        if !self.0.ready_state() {
            return Err(MovementError::DriveNotEnabled(self.0.id));
        }
        self.0
            .set_mode(OperationMode::ProfilePosition)
            .await
            .map_err(MovementError::SetMode)?;
        self.0.unset_control();
        if movement == MovementMode::Relative {
            self.0.set_bit(ControlBit::Control6 as u8, 0);
        }
        self.set_position(target, 3)
            .map_err(MovementError::Ethercat)?;
        self.0.controller.cycle().await;
        self.0.unset_bit(ControlBit::Halt as u8, 0);
        self.0.set_bit(ControlBit::Control4 as u8, 0);
        while !self.0.get_bit(StatusWordBit::AckStartRefReached as u8, 0) {
            self.0.controller.cycle().await;
        }
        while !self.0.get_bit(StatusWordBit::MotionComplete as u8, 0) {
            if self.0.controller.verbose() {
                let id = self.0.id;
                print!(
                    "Move device {id} {movement:?} : {target} {}\r",
                    self.get_position().unwrap()
                );
            }
            self.0.unset_control();
            self.0.controller.cycle().await;
        }
        if self.0.controller.verbose() {
            println!(" completed");
        }
        Ok(())
    }

    pub async fn move_position_velocity(
        &mut self,
        target: i32,
        velocity: u32,
        movement: MovementMode,
    ) -> Result<(), MovementError> {
        self.set_profile_velocity(velocity, 7)
            .map_err(MovementError::Ethercat)?;
        self.move_position(target, movement).await
    }

    pub async fn move_position_velocity_acceleration(
        &mut self,
        target: i32,
        velocity: u32,
        acceleration: u32,
        deceleration: u32,
        movement: MovementMode,
    ) -> Result<(), FullControlMovementError> {
        {
            let sub_device = self
                .0
                .controller
                .group()
                .subdevice(self.0.controller.main_device(), self.0.id)
                .map_err(FullControlMovementError::DeviceNotFound)?;
            sub_device
                .sdo_write(0x6083, 0, acceleration)
                .await
                .map_err(|error| {
                    FullControlMovementError::WritingAccelerationFailed(self.0.id, error)
                })?;
            sub_device
                .sdo_write(0x6084, 0, deceleration)
                .await
                .map_err(|error| {
                    FullControlMovementError::WritingDecelerationFailed(self.0.id, error)
                })?;
        }
        self.move_position_velocity(target, velocity, movement)
            .await
            .map_err(FullControlMovementError::MovementFailed)
    }

    fn set_position(&mut self, target: i32, byte: u8) -> Result<(), EthercrabError> {
        let mut sub_device = self
            .0
            .controller
            .group()
            .subdevice(self.0.controller.main_device(), self.0.id)?;
        for (index, byte) in sub_device.outputs_raw_mut()[usize::from(byte)..]
            .iter_mut()
            .enumerate()
        {
            *byte = (target >> (8 * index)) as u8;
        }
        Ok(())
    }

    fn set_profile_velocity(&mut self, velocity: u32, byte: u8) -> Result<(), EthercrabError> {
        let mut sub_device = self
            .0
            .controller
            .group()
            .subdevice(self.0.controller.main_device(), self.0.id)?;
        for (index, byte) in sub_device.outputs_raw_mut()[usize::from(byte)..]
            .iter_mut()
            .enumerate()
        {
            *byte = (velocity >> (8 * index)) as u8;
        }
        Ok(())
    }
}
