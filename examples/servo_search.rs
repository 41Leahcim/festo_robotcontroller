use std::time::Duration;

use ethercrab::PduStorage;
use festo_robotcontroller::controller::Controller;

static PDU_STORAGE: PduStorage<16, 1100> = PduStorage::new();

fn main() {
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .unwrap();
    let mut controller: Controller<'_, 16, 64> = runtime
        .block_on(Controller::new(
            "eth0",
            Duration::from_millis(20),
            &PDU_STORAGE,
            true,
        ))
        .unwrap();
    for sub_device in controller.device_iter() {
        println!("{sub_device:?}");
    }
}
