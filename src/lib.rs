//! Main module, only contains sub-modules.
#![warn(
    clippy::pedantic,
    clippy::nursery,
    clippy::missing_docs_in_private_items,
    missing_docs
)]
#![allow(clippy::must_use_candidate, clippy::module_name_repetitions)]

#[cfg(not(any(feature = "tokio", feature = "smol")))]
const _: () = panic!("Either tokio or smol needs to be used as async runtime");

pub mod controller;
pub mod device;
