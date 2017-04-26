extern crate nalgebra;
extern crate simplesvg;
extern crate rand;
extern crate getopts;
extern crate num_cpus;
extern crate rayon;
extern crate regex;
extern crate tempfile;

pub mod geom;
pub mod geom_import;
pub mod trace;
pub mod render;
pub mod parcombs;
pub mod app;

use std::env;
use std::io;
use std::io::Write;
use std::process;

// See http://stackoverflow.com/questions/30281235/how-to-cleanly-end-the-program-with-an-exit-code
// Having this as a separate function ensures that all destructors etc. are run before process
// exits.
fn real_main() -> i32 {
    match app::parse_command_line(&(env::args().collect())) {
        Err(e) => {
            write!(io::stderr(), "{}\n", e).unwrap();
            return -1;
        }
        Ok(params) => {
            if let Err(e) = app::do_run(&params) {
                writeln!(&mut io::stderr(), "{}\n", e).unwrap();
                return -1;
            }
        }
    }
    0
}

fn main() {
    process::exit(real_main());
}