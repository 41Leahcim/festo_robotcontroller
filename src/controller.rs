//! This module contains everything related to the controller.
//! The controller initializes the network and allowes devices to communicate and be created.

use core::{
    fmt::{self, Debug, Formatter},
    time::Duration,
};
use std::{io, time::Instant};

use ethercrab::{
    error::Error as EthercrabError,
    std::{ethercat_now, tx_rx_task},
    MainDevice, MainDeviceConfig, PduStorage, SubDeviceGroup,
};

/// An error occured while creating a controller
pub enum ControllerError {
    /// Somehting went wrong while trying to communicate over Ethercat
    Ethercat(EthercrabError),

    /// Something went wrong while setting the cycle time
    CycleTime(EthercrabError),

    /// Something went wrong while setting the output pdo's
    OutputPdo(EthercrabError),

    /// Something went wrong while seeting the input pdo's
    InputPdo(EthercrabError),

    /// There is already an active `Controller` for the `PduStorage`
    AnotherControllerExists,

    /// Failed to spawn a task to send and receive data
    FailedToSpawnUpdateTask(io::Error),

    /// Failed to initialize the devicegroup for this controller
    FailedToInitializeGroup(EthercrabError),

    /// Failed to make the devicegroup operational
    FailedToMakeGroupOperational(EthercrabError),
}

impl Debug for ControllerError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            Self::Ethercat(error) => write!(f, "Error while configuring network: {error}"),
            Self::CycleTime(error) => write!(f, "Failed to set cycle time: {error}"),
            Self::OutputPdo(error) => write!(f, "Failed to set output pdo's: {error}"),
            Self::InputPdo(error) => write!(f, "Failed to set input pdo's: {error}"),
            Self::AnotherControllerExists => write!(f, "Only one controller can exist at any time"),
            Self::FailedToSpawnUpdateTask(error) => {
                write!(f, "Failed to spawn update task: {error}")
            }
            Self::FailedToInitializeGroup(error) => {
                write!(f, "Failed to initialize group: {error}")
            }
            Self::FailedToMakeGroupOperational(error) => {
                write!(f, "Failed to make group operational: {error}")
            }
        }
    }
}

/// The controller struct, only 1 is allowed to exist at any time.
/// Recommended constants:
/// - `MAX_DEVICES`: 16
/// - `PDI_LENGTH`: 64
pub struct Controller<'main_device, const MAX_DEVICES: usize, const PDI_LENGTH: usize> {
    /// The duration of a cycle
    cycle_time: Duration,

    /// Whether to log info messages
    verbose: bool,

    /// The main device
    main_device: MainDevice<'main_device>,

    /// The group of connected devices
    group: SubDeviceGroup<MAX_DEVICES, PDI_LENGTH, ethercrab::subdevice_group::Op>,
}

impl<const MAX_DEVICES: usize, const PDI_LENGTH: usize> Controller<'_, MAX_DEVICES, PDI_LENGTH> {
    /// Returns the cycle time specified at construction time.
    /// Updates will always take atleast this amount of time.
    pub const fn cycle_time(&self) -> Duration {
        self.cycle_time
    }

    /// Returns whether information is logged.
    pub const fn verbose(&self) -> bool {
        self.verbose
    }

    /// Returns a reference to the main device, used for communicating with slaves
    pub const fn main_device(&self) -> &MainDevice {
        &self.main_device
    }

    /// Returns a reference to the group containing all devices to communicate with
    pub const fn group(
        &self,
    ) -> &SubDeviceGroup<MAX_DEVICES, PDI_LENGTH, ethercrab::subdevice_group::Op> {
        &self.group
    }

    /// Returns an iterator over the connected devices.
    pub fn device_iter(
        &mut self,
    ) -> ethercrab::GroupSubDeviceIterator<
        '_,
        '_,
        MAX_DEVICES,
        PDI_LENGTH,
        ethercrab::subdevice_group::Op,
        ethercrab::subdevice_group::NoDc,
    > {
        self.group.iter(&self.main_device)
    }

    /// Configures the subdevices
    async fn configure_devices(
        group: &mut SubDeviceGroup<MAX_DEVICES, PDI_LENGTH>,
        main_device: &MainDevice<'_>,
        cycle_time: Duration,
        verbose: bool,
    ) -> Result<(), ControllerError> {
        /// The index to write the output PDO's to
        const PDO_OUTPUT_INDEX: u16 = 0x1600;

        /// The values for the output PDO's
        const PDO_OUTPUT: [u32; 9] = [
            0x6040_0010,
            0x6060_0008,
            0x607a_0020,
            0x6081_0020,
            0x60ff_0020,
            0x6071_0010,
            0x60b1_0020,
            0x60b2_0010,
            0x0000_0008,
        ];

        /// The index to write the input PDO's to
        const PDO_INPUT_INDEX: u16 = 0x1A00;

        /// The values for the input PDO's
        const PDO_INPUT: [u32; 7] = [
            0x6041_0010,
            0x6061_0008,
            0x6064_0020,
            0x606c_0020,
            0x6077_0010,
            0x2194_0520,
            0x0000_0008,
        ];

        for sub_device in group.iter(main_device) {
            log::info!("Configuring device {}", sub_device.identity());
            // Check if name or eeprom-id is correct for all types of CMMT
            if !matches!(sub_device.name(), "CMMT-AS" | "CMMT-ST")
                && !matches!(sub_device.identity().product_id, 0x7B_5A25 | 0x7B_1A95)
            {
                continue;
            }

            if verbose {
                log::info!(
                    "Doing Cia402 configuration for device {}",
                    sub_device.identity()
                );
            }
            // Everything cycle time related (should be checked)
            sub_device
                .sdo_write(0x212E, 2, cycle_time.as_secs_f32())
                .await
                .map_err(ControllerError::CycleTime)?;

            // Set output PDOs
            sub_device
                .sdo_write_array(PDO_OUTPUT_INDEX, &PDO_OUTPUT)
                .await
                .map_err(ControllerError::OutputPdo)?;

            // Set input PDOs
            sub_device
                .sdo_write_array(PDO_INPUT_INDEX, &PDO_INPUT)
                .await
                .map_err(ControllerError::InputPdo)?;

            // Configure the servo controller
            /*sub_device
                .sdo_write(0x1C12, 1, 0x1600u16)
                .await
                .map_err(ControllerError::Ethercat)?;
            sub_device
                .sdo_write(0x1C13, 1, 0x1A00u16)
                .await
                .map_err(ControllerError::Ethercat)?;
            sub_device
                .sdo_write(0x1C12, 0, 1u8)
                .await
                .map_err(ControllerError::Ethercat)?;
            sub_device
                .sdo_write(0x1C13, 0, 1u8)
                .await
                .map_err(ControllerError::Ethercat)?;*/

            if verbose {
                log::info!("Done mapping drive");
            }
        }
        Ok(())
    }

    /// Creates the controller struct, used to create and communicate with devices.
    /// Only one controller can exist at any time.
    ///
    /// Recommanded values for constants:
    /// - `MAX_FRAMES`: 16
    /// - `MAX_PDU_DATA`: 1100
    ///
    /// # Errors
    /// Returns an error if:
    /// - There already is a controller
    /// - The group couldn't be initialized
    /// - A slave couldn't be configured
    /// - The group couldn't be made operational
    ///
    /// # Returns
    /// The `controller` struct or an error
    pub async fn new<const MAX_FRAMES: usize, const MAX_PDU_DATA: usize>(
        interface_name: &str,
        cycle_time: Duration,
        pdu_storage: &'static PduStorage<MAX_FRAMES, MAX_PDU_DATA>,
        verbose: bool,
    ) -> Result<Self, ControllerError> {
        if verbose {
            log::info!("Starting Ethercat Master");
        }

        // Initialize SOEM, bind socket to `interface_name`
        let (tx, rx, pdu_loop) = pdu_storage
            .try_split()
            .map_err(|()| ControllerError::AnotherControllerExists)?;
        let main_device = MainDevice::new(
            pdu_loop,
            ethercrab::Timeouts {
                wait_loop_delay: Duration::from_millis(2),
                mailbox_response: Duration::from_millis(1_000),
                ..Default::default()
            },
            MainDeviceConfig::default(),
        );

        let update_task =
            tx_rx_task(interface_name, tx, rx).map_err(ControllerError::FailedToSpawnUpdateTask)?;
        #[cfg(feature = "tokio")]
        tokio::spawn(update_task);
        #[cfg(feature = "smol")]
        smol::spawn(update_task).detach();

        let mut group = main_device
            .init_single_group::<MAX_DEVICES, PDI_LENGTH>(ethercat_now)
            .await
            .map_err(ControllerError::FailedToInitializeGroup)?;

        if verbose {
            log::info!("Context initialization on {interface_name:?} succeeded.");
        }

        Self::configure_devices(&mut group, &main_device, cycle_time, verbose).await?;

        let group = Box::pin(group.into_op(&main_device))
            .await
            .map_err(ControllerError::FailedToMakeGroupOperational)?;

        Ok(Self {
            cycle_time,
            verbose,
            main_device,
            group,
        })
    }

    /// Updates the device state, takes at least the at construction specified cycle time.
    /// If the update takes shorter than the specified time, the thread will sleep.
    /// If the update takes longer, a warning message will be displayed.
    pub async fn cycle(&self) {
        let start = Instant::now();
        let _ = self.group.tx_rx_sync_system_time(&self.main_device).await;
        let delta = start.elapsed();
        if delta > self.cycle_time {
            log::info!(
                "System too slow for cycle time {:?} sending takes {delta:?}",
                self.cycle_time
            );
        } else {
            let sleep_duration = self.cycle_time - delta;
            #[cfg(feature = "tokio")]
            tokio::time::sleep(sleep_duration).await;
            #[cfg(feature = "smol")]
            smol::unblock(move || std::thread::sleep(sleep_duration)).await;
        }
    }
}
