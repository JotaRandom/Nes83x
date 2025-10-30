//! NES83x - A NES emulator written in Rust

use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use dialoguer::{theme::ColorfulTheme, Select};
use log::{error, info, warn};
use simplelog::{ColorChoice, Config, LevelFilter, TerminalMode, TermLogger};
use std::path::{Path, PathBuf};

mod emulator;
mod nes;
mod rom_loader;
pub mod utils;

use emulator::Emulator;
use rom_loader::{load_rom, list_roms, RomError};

/// Command-line arguments
#[derive(Parser, Debug)]
#[command(version, about, long_about = None)]
struct Args {
    /// ROM file to load
    rom: Option<PathBuf>,
    
    /// Directory containing ROMs (default: ./roms)
    #[arg(short, long, default_value = "roms")]
    rom_dir: PathBuf,
    
    /// List available ROMs and exit
    #[arg(short, long)]
    list: bool,
    
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
    let mut emulator = Emulator::new()?;
    
    // If --list was specified, list ROMs and exit
    if args.list {
        list_and_exit(&args.rom_dir)?;
    }
    
    // If a ROM was specified, load it
    if let Some(rom_path) = args.rom {
        load_and_run(&mut emulator, &rom_path)?;
    } else {
        // No ROM specified, show interactive menu
        interactive_menu(&mut emulator, &args.rom_dir)?;
    }
    
    // Start the emulator's main loop
    emulator.run().map_err(|e| anyhow::anyhow!(e))
}

/// List available ROMs and exit
fn list_and_exit(rom_dir: &Path) -> Result<()> {
    println!("Available ROMs in {}:", rom_dir.display());
    
    match list_roms(rom_dir) {
        Ok(roms) => {
            if roms.is_empty() {
                println!("  No ROMs found");
            } else {
                for rom in roms {
                    println!("  {}", rom);
                }
            }
        }
        Err(e) => {
            eprintln!("Error reading ROM directory: {}", e);
        }
    }
    
    std::process::exit(0);
}

/// Show interactive menu to select and load a ROM
fn interactive_menu(emulator: &mut Emulator, rom_dir: &Path) -> Result<()> {
    let roms = match list_roms(rom_dir) {
        Ok(roms) => roms,
        Err(e) => {
            eprintln!("Error reading ROM directory: {}", e);
            return Ok(());
        }
    };
    
    if roms.is_empty() {
        println!("No ROMs found in {}", rom_dir.display());
        println!("Please place .nes files in the roms/ directory.");
        return Ok(());
    }
    
    let selection = Select::with_theme(&ColorfulTheme::default())
        .with_prompt("Select a ROM to run")
        .default(0)
        .items(&roms)
        .interact()?;
    
    let rom_path = rom_dir.join(&roms[selection]);
    load_and_run(emulator, &rom_path)
}

/// Load a ROM and start the emulator
fn load_and_run(emulator: &mut Emulator, rom_path: &Path) -> Result<()> {
    info!("Loading ROM: {}", rom_path.display());
    
    match emulator.load_rom(rom_path) {
        Ok(_) => {
            info!("ROM loaded successfully");
            emulator.start()?;
            
            // Run the main loop
            if let Err(e) = emulator.run() {
                error!("Emulator error: {}", e);
                return Err(e).context("Emulator error");
            }
            
            info!("Shutting down NES83x");
            Ok(())
        }
        Err(e) => {
            error!("Failed to load ROM: {}", e);
            Err(e).context("Failed to load ROM")
        }
    }
}
