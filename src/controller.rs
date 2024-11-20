use std::{
    fmt::Debug,
    sync::Arc,
    time::{Duration, Instant},
};

use ethercrab::{
    std::{ethercat_now, tx_rx_task},
    MainDevice, MainDeviceConfig, PduStorage, SubDeviceGroup,
};

pub enum ControllerError {
    NoSocketConnection(String),
    FailedToStartMaster,
    Ethercat(ethercrab::error::Error),
}

impl Debug for ControllerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self{
            Self::NoSocketConnection(interface) => write!(f, "No socket connection on {interface}\nExecute as root and verify your network adapter name"),
            Self::FailedToStartMaster => write!(f, "Failed to start master"),
            Self::Ethercat(error) => write!(f, "{error}")
        }
    }
}

#[derive(Debug)]
pub struct Timeout;

pub struct MapCia402Error(u16);

impl Debug for MapCia402Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "Check PDO mapping on device {}", self.0)
    }
}

const MAX_FRAMES: usize = 16;
const MAX_PDU_DATA: usize = 1100;
const MAX_DEVICES: usize = 16;
const PDI_LENGTH: usize = 64;

static PDU_STORAGE: PduStorage<MAX_FRAMES, MAX_PDU_DATA> = PduStorage::new();

pub struct Controller<'main_device> {
    cycle_time: Duration,
    verbose: bool,
    main_device: Arc<MainDevice<'main_device>>,
    group: SubDeviceGroup<MAX_DEVICES, PDI_LENGTH, ethercrab::subdevice_group::Op>,
}

impl Controller<'_> {
    pub const fn cycle_time(&self) -> Duration {
        self.cycle_time
    }

    pub const fn verbose(&self) -> bool {
        self.verbose
    }

    pub fn main_device(&self) -> &MainDevice {
        &self.main_device
    }

    pub const fn group(
        &self,
    ) -> &SubDeviceGroup<MAX_DEVICES, PDI_LENGTH, ethercrab::subdevice_group::Op> {
        &self.group
    }

    pub async fn new(
        interface_name: &str,
        cycle_time: Duration,
        verbose: bool,
    ) -> Result<Self, ControllerError> {
        if verbose {
            println!("Starting Ethercat Master");
        }

        // Initialize SOEM, bind socket to `interface_name`
        let (tx, rx, pdu_loop) = PDU_STORAGE.try_split().expect("Can only split once");
        let main_device = Arc::new(MainDevice::new(
            pdu_loop,
            ethercrab::Timeouts {
                wait_loop_delay: Duration::from_millis(2),
                mailbox_response: Duration::from_millis(1_000),
                ..Default::default()
            },
            MainDeviceConfig::default(),
        ));
        tokio::spawn(tx_rx_task(interface_name, tx, rx).expect("Spawn TX/RX task"));

        let mut group = main_device
            .init_single_group::<MAX_DEVICES, PDI_LENGTH>(ethercat_now)
            .await
            .expect("Failed to initialize");

        if verbose {
            println!("Context initialization on {interface_name:?} succeeded.");
        }

        for sub_device in group.iter(&main_device) {
            println!("Configuring device {}", sub_device.identity());
            // Check if name or eeprom-id is correct for all types of CMMT
            if !matches!(sub_device.name(), "CMMT-AS" | "CMMT-ST")
                && !matches!(sub_device.identity().product_id, 0x7B5A25 | 0x7B1A95)
            {
                continue;
            }
            // Set output PDOs
            const PDO_OUTPUT_INDEX: u16 = 0x1600;
            const PDO_OUTPUT: [u32; 9] = [
                0x60400010, 0x60600008, 0x607a0020, 0x60810020, 0x60ff0020, 0x60710010, 0x60b10020,
                0x60b20010, 0x00000008,
            ];
            const PDO_INPUT_INDEX: u16 = 0x1A00;
            const PDO_INPUT: [u32; 7] = [
                0x60410010, 0x60610008, 0x60640020, 0x606c0020, 0x60770010, 0x21940520, 0x00000008,
            ];
            if verbose {
                println!(
                    "Doing Cia402 configuration for device {}",
                    sub_device.identity()
                );
            }
            // Everything cycle time related (should be checked)
            sub_device
                .sdo_write(0x212E, 2, cycle_time.as_secs_f32())
                .await
                .map_err(ControllerError::Ethercat)?;

            // Set output PDOs
            sub_device
                .sdo_write_array(PDO_OUTPUT_INDEX, &PDO_OUTPUT)
                .await
                .map_err(ControllerError::Ethercat)?;

            // Set input PDOs
            sub_device
                .sdo_write_array(PDO_INPUT_INDEX, &PDO_INPUT)
                .await
                .map_err(ControllerError::Ethercat)?;

            // Configure the servo controller
            sub_device
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
                .map_err(ControllerError::Ethercat)?;

            if verbose {
                println!("Done mapping drive");
            }
        }

        let group = group.into_op(&main_device).await.unwrap();

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
            println!(
                "System too slow for cycle time {:?} sending takes {delta:?}",
                self.cycle_time
            );
        } else {
            tokio::time::sleep(self.cycle_time - delta).await;
        }
    }
}
