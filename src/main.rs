//! NES83x - A NES emulator written in Rust

use anyhow::{Context, Result};
use clap::Parser;
use log::{error, info};
use simplelog::{ColorChoice, Config, LevelFilter, TerminalMode, TermLogger};
use std::path::PathBuf;

mod cpu;
mod emulator;
mod nes;
mod utils;

use emulator::Emulator;

/// Command-line arguments
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// ROM file to load
    rom: Option<PathBuf>,
    
    /// Enable debug logging
    #[arg(short, long)]
    debug: bool,
    
    /// Scale factor for the window (default: 2)
    #[arg(short, long, default_value_t = 2)]
    scale: u32,
}

fn main() -> Result<()> {
    // Parse command-line arguments
    let args = Args::parse();
    
    // Initialize logging
    let log_level = if args.debug {
        LevelFilter::Debug
    } else {
        LevelFilter::Info
    };
    
    TermLogger::init(
        log_level,
        Config::default(),
        TerminalMode::Stderr,
        ColorChoice::Auto,
    )?;
    
    info!("Starting NES83x emulator");
    
    // Create and initialize the emulator
    let mut emulator = Emulator::new()
        .context("Failed to create emulator")?;
    
    emulator.init()
        .context("Failed to initialize emulator")?;
    
    // Load ROM if specified
    if let Some(rom_path) = args.rom {
        info!("Loading ROM: {}", rom_path.display());
        
        if let Err(e) = emulator.load_rom(&rom_path) {
            error!("Failed to load ROM: {}", e);
            return Err(e).context("Failed to load ROM");
        }
    }
    
    // Start the emulator
    if let Err(e) = emulator.start() {
        error!("Failed to start emulator: {}", e);
        return Err(e).context("Failed to start emulator");
    }
    
    // Run the main loop
    if let Err(e) = emulator.run() {
        error!("Emulator error: {}", e);
        return Err(e).context("Emulator error");
    }
    
    info!("Shutting down NES83x");
    Ok(())
}
