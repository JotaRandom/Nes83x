use anyhow::Result;
use clap::Parser;
use log::{error, info};
use simplelog::{ColorChoice, Config, LevelFilter, TermLogger, TerminalMode};
use std::path::PathBuf;

use nes83x::emulator::minifb::Emulator;

/// Command-line arguments
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// ROM file to load (optional - GUI will be shown if not provided)
    rom: Option<PathBuf>,

    /// Directory containing ROMs (default: roms)
    #[arg(short, long, default_value = "roms")]
    rom_dir: PathBuf,

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

    info!("Starting NES83x minifb emulator");

    // Create the minifb emulator frontend
    let mut emulator = match Emulator::new() {
        Ok(emulator) => emulator,
        Err(e) => {
            error!("Failed to create minifb emulator: {}", e);
            error!("Make sure your system supports window creation");
            return Err(anyhow::anyhow!("minifb initialization failed: {}", e));
        }
    };

    // Initialize the UI
    if let Err(e) = emulator.init() {
        error!("Failed to initialize emulator UI: {}", e);
        return Err(anyhow::anyhow!("UI initialization failed: {}", e));
    }

    // If a ROM was specified via command line, load it
    if let Some(rom_path) = args.rom {
        info!("Loading ROM from command line: {}", rom_path.display());
        if let Err(e) = emulator.load_rom(rom_path) {
            error!("Failed to load ROM: {}", e);
            return Err(anyhow::anyhow!("ROM loading failed: {}", e));
        }
    }
    // Note: Auto-loading from roms/ directory disabled to start without ROM for debugging

    if emulator.rom_path.is_some() {
        info!("ROM loaded: {:?}", emulator.rom_path);
    } else {
        info!("No ROM loaded, starting with placeholder");
    }

    // Run the emulator
    if let Err(e) = emulator.run() {
        error!("Emulator error: {}", e);
        return Err(anyhow::anyhow!("Emulator runtime error: {}", e));
    }

    Ok(())
}