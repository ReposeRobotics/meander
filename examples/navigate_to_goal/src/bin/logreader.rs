//! Extracts the recorded CopperLists from a navigate-to-goal log:
//! `cargo run -p meander-example-navigate-to-goal --bin logreader -- target/navigate-to-goal/log.copper extract-copperlists`

use cu29::prelude::*;
use cu29_export::run_cli;

gen_cumsgs!("copperconfig.ron");

fn main() {
    run_cli::<CuMsgs>().expect("logreader CLI failed");
}
