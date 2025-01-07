use std::{hint::spin_loop, io, time::Duration};

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

async fn perform_movement<const MAX_DEVICES: usize, const PDI_LENGTH: usize>(
    servo: &mut Servo<'_, '_, MAX_DEVICES, PDI_LENGTH>,
    buffer: &mut String,
) {
    loop {
        // Read the position to move to, it's still possible to stop
        println!("Enter position: ");
        buffer.clear();
        io::stdin().read_line(buffer).expect("Failed to read input");
        let Ok(position) = buffer.trim().parse::<i32>() else {
            eprintln!("Invalid position, returning to mode selection!");
            break;
        };

        // Request the movement method
        println!("Enter 0 for absolute, enter 1 for relative:");
        buffer.clear();
        io::stdin()
            .read_line(buffer)
            .expect("Failed  to read input");
        let Some(movement) = buffer
            .trim()
            .parse::<u8>()
            .ok()
            .filter(|value| matches!(value, 0 | 1))
            .map(|value| {
                if value == 0 {
                    MovementMode::Absolute
                } else {
                    MovementMode::Relative
                }
            })
        else {
            eprintln!("Invalid input, returning to position reading");
            continue;
        };

        // Request the velocity to move at, move at normal velocity on failure
        println!("Enter velocity: ");
        buffer.clear();
        io::stdin().read_line(buffer).expect("Failed to read input");
        let Ok(velocity) = buffer.trim().parse::<u32>() else {
            println!("Failed to read velocity, using default.");
            if let Err(error) = servo.move_position(position, movement).await {
                eprintln!("Failed to move servo: {error:?}");
            }
            break;
        };

        // Request the acceleration, move at default acceleration and deceleration on failure
        println!("Enter acceleration: ");
        buffer.clear();
        let Ok(acceleration) = buffer.trim().parse::<u32>() else {
            println!("Failed to read acceleration, using default.");
            if let Err(error) = servo
                .move_position_velocity(position, velocity, movement)
                .await
            {
                eprintln!("Failed to move servo: {error:?}");
            }
            break;
        };

        // Request the deceleration, move at default acceleration and deceleration on failure
        let Ok(deceleration) = buffer.trim().parse::<u32>() else {
            eprintln!("Failed to read deceleration, using default acceleration and deceleration.");
            if let Err(error) = servo
                .move_position_velocity(position, velocity, movement)
                .await
            {
                eprintln!("Failed to move servo: {error:?}");
            }
            break;
        };

        // Move as requested
        if let Err(error) = servo
            .move_position_velocity_acceleration(
                position,
                velocity,
                acceleration,
                deceleration,
                movement,
            )
            .await
        {
            eprintln!("Failed to move servo: {error:?}");
        }
        break;
    }
}

async fn perform_action<const MAX_DEVICES: usize, const PDI_LENGTH: usize>(
    servo: &mut Servo<'_, '_, MAX_DEVICES, PDI_LENGTH>,
    action: u8,
    buffer: &mut String,
) {
    // Perform the specified movement with required error handling
    match action {
        0 => servo.jog_stop().await,
        1 => {
            if let Err(error) = servo.jog_negative().await {
                eprintln!("Failed to jog negative: {error:?}")
            }
        }
        2 => {
            if let Err(error) = servo.jog_positive().await {
                eprintln!("Failed to jog positive: {error:?}")
            }
        }
        3 => {
            if let Err(error) = servo.home(false).await {
                eprintln!("Failed to move home: {error:?}")
            }
        }
        4 => perform_movement(servo, buffer).await,
        _ => println!("Invalid option, try again"),
    }
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
            true,
        )
        .await
        .expect("Failed to initialize controller");
        eprintln!("Initialized controller");

        // Find the first device matching the requested properties
        let servo_number = find_device(&mut controller, args)
            .await
            .expect("Failed to find device");
        eprintln!("Found device");

        // Treat the found device as a servo
        let mut servo = Servo::new(&controller, servo_number)
            .await
            .expect("Failed to initialize device");
        eprintln!("Device can be used as servo");
        loop {
            spin_loop();
        }

        // Create an input buffer
        let mut buffer = String::new();
        loop {
            // Read the movement action
            println!("Possible movements:");
            println!(" 0. Jog stop");
            println!(" 1. Jog negative");
            println!(" 2. Jog negative");
            println!(" 3. Home");
            println!(" 4. Move");
            println!("Enter the number for the movement for the servo to make:");
            buffer.clear();
            io::stdin()
                .read_line(&mut buffer)
                .expect("Failed to read input");

            // Parse the movement code
            let Ok(selection) = buffer.trim().parse::<u8>() else {
                println!("Make sure to enter the number next to the movement you want.");
                continue;
            };
            perform_action(&mut servo, selection, &mut buffer).await;
        }
    });
}
