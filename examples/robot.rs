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
static PDU_STORAGE: PduStorage<16, 1100> = PduStorage::new();

/// This struct parces and stores the arguments passed to the application.
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
        .expect("Failed to create tokio runtime");

    // Start the runtime
    runtime.block_on(async {
        // Create a new controller
        let controller: Controller<'_, 16, 64> = Controller::new(
            &args.interface,
            Duration::from_millis(20),
            &PDU_STORAGE,
            false,
        )
        .await
        .expect("Failed to initialize controller");

        // Find the first device matching the requested properties
        let controller = Arc::new(controller);

        for (number, name) in [
            (0, "horizontal_rotation"),
            (1, "bender"),
            (2, "vertical_rotation"),
            (3, "object_rotation"),
        ] {
            // Treat the found device as a servo
            let mut servo = Servo::new(&controller, number)
                .await
                .unwrap_or_else(|error| panic!("Failed to connect to {name} motor: {error:?}"));
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
        }
    });
}
