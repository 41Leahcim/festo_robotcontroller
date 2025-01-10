use std::{hint::spin_loop, sync::Arc, time::Duration};

use clap::Parser;
use ethercrab::PduStorage;
use festo_robotcontroller::{
    controller::Controller,
    device::servo::{MovementMode, Servo},
};

static PDU_STORAGE: PduStorage<16, 1100> = PduStorage::new();

#[derive(Debug, Parser)]
pub struct Args {
    /// The interface to communicate over
    interface: String,

    #[arg(short)]
    device_number: Option<usize>,

    #[arg(short)]
    alias_address: Option<u16>,

    #[arg(short)]
    configured_address: Option<u16>,

    #[arg(short)]
    description: Option<String>,
}

fn none_or_equal<T: PartialEq>(left: Option<T>, right: T) -> bool {
    left.is_none() || left == Some(right)
}

async fn find_device<const MAX_DEVICES: usize, const PDI_LENGTH: usize>(
    controller: &mut Controller<'_, MAX_DEVICES, PDI_LENGTH>,
    args: Args,
) -> Option<usize> {
    if let Some(device_number) = args.device_number {
        let device = controller
            .group()
            .subdevice(controller.main_device(), device_number)
            .expect("Failed to take control of device");
        assert!(none_or_equal(args.alias_address, device.alias_address()));
        assert!(none_or_equal(
            args.configured_address,
            device.configured_address()
        ));
        return Some(device_number);
    }
    for (index, device) in controller.device_iter().enumerate() {
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

    runtime.block_on(async {
        // Create a new controller
        let mut controller: Controller<'_, 16, 64> = Controller::new(
            &args.interface,
            Duration::from_millis(20),
            &PDU_STORAGE,
            false,
        )
        .await
        .expect("Failed to initialize controller");
        eprintln!("Initialized controller");

        // Find the first device matching the requested properties
        let servo_number = find_device(&mut controller, args)
            .await
            .expect("Failed to find device");
        eprintln!("Found device");
        let controller = Arc::new(controller);

        // Treat the found device as a servo
        let mut servo = Servo::new(&controller, servo_number)
            .await
            .expect("Failed to initialize device");
        eprintln!("Device can be used as servo");

        servo.home(true).await.unwrap();
        servo
            .move_position(10_000, MovementMode::Absolute)
            .await
            .unwrap();
        servo
            .move_position(0, MovementMode::Absolute)
            .await
            .unwrap();
        loop {
            spin_loop();
        }
    });
}
