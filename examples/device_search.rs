//! Scans the Ethercat network for devices.
//! On success it displays information about all found devices.

use std::time::Duration;

use clap::Parser;
use ethercrab::PduStorage;
use festo_robotcontroller::controller::Controller;

static PDU_STORAGE: PduStorage<16, 1100> = PduStorage::new();

#[derive(Debug, Parser)]
pub struct Args {
    /// The interface to communicate over
    interface: String,
}

fn main() {
    // Parse the arguments
    let args = Args::parse();

    // Create a new single threaded runtime
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();

    // Run the async code
    runtime.block_on(async {
        // Create a new controller
        let mut controller: Controller<'_, 16, 64> = Controller::new(
            &args.interface,
            Duration::from_millis(20),
            &PDU_STORAGE,
            true,
        )
        .await
        .unwrap();

        // Iterate over the connected devices
        for (index, sub_device) in controller.device_iter().enumerate() {
            // Display the device number and address
            println!("Device number: {index}");
            println!("Alias address: {}", sub_device.alias_address());
            println!("Configured address: {}", sub_device.configured_address());

            // Try to retrieve the full name of the device
            if let Ok(Some(description)) = sub_device.description().await {
                // On success, display the full name
                println!("Device type: {description}",);
            } else {
                // Otherwise, display the partial name
                println!("Model name: {}", sub_device.name());
            }

            // Display the identity of the device (vendor, product, revision, serial numbers)
            println!("Device identity: ({})", sub_device.identity());

            // Use an empty line between devices
            println!();
        }
    });
}
