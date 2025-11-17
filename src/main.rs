use anyhow::Result;
use clap::Parser;
use log::{error, info};
use simplelog::{ColorChoice, Config, LevelFilter, TermLogger, TerminalMode};
use std::path::PathBuf;

use nes83x::nes::Nes;
use nes83x::rom_loader;

/// Command-line arguments
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// ROM file to load (optional - CLI will run frames if provided)
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

    info!("Starting NES83x CLI emulator");

    if let Some(rom_path) = args.rom {
        info!("Loading ROM: {}", rom_path.display());

        let mut nes = Nes::new();
        nes.load_rom(&rom_path)?;

        info!("Running 100 frames...");
        for i in 0..100 {
            nes.run_frame()?;
            if i % 10 == 0 {
                info!("Frame {}", nes.ppu.borrow().frame());
            }
        }
        info!("CLI emulation finished");
    } else {
        info!("No ROM specified. Use --help for options or provide a ROM path");
    }

    Ok(())
}
