#![warn(missing_docs, clippy::missing_docs_in_private_items)]
//! This application controls the selected servo or stepper motor.

use clap::Parser;
use ethercrab::PduStorage;
use festo_robotcontroller::{
    controller::Controller,
    device::servo::{MovementMode, Servo},
};
use std::{
    sync::Arc,
    time::{Duration, Instant},
};

/// This will store the PDU messages until sent
static PDU_STORAGE: PduStorage<128, 1100> = PduStorage::new();

/// This struct parces and stores the arguments passed to the application.
#[derive(Debug, Parser)]
pub struct Args {
    /// The interface to communicate over
    interface: String,

    /// The number of the device to control, can be found with device search
    /// This value is unique and will thus be found the fastest.
    #[arg(short = 'n')]
    device_number: Option<usize>,

    /// The alias address of the device to control, can be found with device search
    #[arg(short)]
    alias_address: Option<u16>,

    /// The configured address of the device to control, can be found with device search
    #[arg(short)]
    configured_address: Option<u16>,

    /// The description of the device to control, can be found with device search
    #[arg(short)]
    description: Option<String>,
}

/// Checks whether the left argument is None or contains the same value as the right argument
fn none_or_equal<T: PartialEq>(left: Option<T>, right: T) -> bool {
    left.is_none() || left == Some(right)
}

/// Searches for the selected device, returns the first device found.
async fn find_device<const MAX_DEVICES: usize, const PDI_LENGTH: usize>(
    controller: &mut Controller<'_, MAX_DEVICES, PDI_LENGTH>,
    args: Args,
) -> Option<usize> {
    // Check whether the device number argument has been passed
    if let Some(device_number) = args.device_number {
        // If so, select the requested device
        let device = controller
            .group()
            .subdevice(controller.main_device(), device_number)
            .expect("Failed to take control of device");

        // Make sure the device conforms to the other selected options if any.
        assert!(none_or_equal(args.alias_address, device.alias_address()));
        assert!(none_or_equal(
            args.configured_address,
            device.configured_address()
        ));

        // Return the device number
        return Some(device_number);
    }

    // Iterate through the connected devices
    for (index, device) in controller.device_iter().enumerate() {
        // Check whether the device has all the requested properties, skip if not.
        // The description will be ignored if the current device doesn't have a description.
        if !none_or_equal(args.alias_address, device.alias_address()) {
            continue;
        }
        if !none_or_equal(args.configured_address, device.configured_address()) {
            continue;
        }
        if let Ok(Some(description)) = device.description().await {
            if !none_or_equal(args.description.as_deref(), &description) {
                continue;
            }
        }

        // The requested device has been found, return the device number
        return Some(index);
    }
    None
}

fn main() {
    // Parse the arguments
    let args = Args::parse();

    // Create a new single threaded runtime
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .expect("Failed to create tokio runtime");

    // Start the runtime
    runtime.block_on(async {
        // Create a new controller
        let mut controller: Controller<'_, 16, 128> = Controller::new(
            &args.interface,
            Duration::from_millis(20),
            &PDU_STORAGE,
            false,
        )
        .await
        .expect("Failed to initialize controller");

        // Find the first device matching the requested properties
        let servo_number = find_device(&mut controller, args)
            .await
            .expect("Failed to find device");
        let controller = Arc::new(controller);

        // Treat the found device as a servo
        let mut servo = Servo::new(&controller, servo_number)
            .await
            .expect("Failed to initialize device");

        // Move the motor to the home position
        eprintln!("Homing");
        servo.home(true).await.unwrap();

        // Move the motor in the positive direction
        eprintln!("Moving in positive direction");
        servo
            .move_position_velocity_acceleration(190_000, 100, 10, 10, MovementMode::Absolute)
            .await
            .unwrap();

        // Move the motor in the negative direction
        eprintln!("Moving in negative direction");
        servo
            .move_position(-1_000, MovementMode::Absolute)
            .await
            .unwrap();

        // Move the motor back to 0
        eprintln!("Moving to 0 position");
        servo
            .move_position(0, MovementMode::Absolute)
            .await
            .unwrap();

        // Jog in positive direction
        eprintln!("Jogging in positive direction");
        servo.jog_positive().await.unwrap();
        let mut jog_start = Instant::now();
        while jog_start.elapsed().as_secs() < 4 {
            controller.cycle().await;
        }

        // Jog back
        jog_start = Instant::now();
        eprintln!("Jogging in negative direction");
        servo.jog_negative().await.unwrap();
        while jog_start.elapsed().as_secs() < 4 {
            controller.cycle().await;
        }
        servo.jog_stop().await;

        servo.disable().await.unwrap();
    });
}
