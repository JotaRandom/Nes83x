//! ROM loading and management for the NES emulator

use std::fs::File;
use std::io::Read;
use std::path::Path;
use zip::result::ZipError;
use zip::ZipArchive;
use thiserror::Error;

use crate::nes::{Nes, NesError};

/// Errors that can occur during ROM loading
#[derive(Error, Debug)]
pub enum RomError {
    #[error("I/O error: {0}")]
    IoError(#[from] std::io::Error),
    
    #[error("ZIP error: {0}")]
    ZipError(#[from] ZipError),
    
    #[error("NES error: {0}")]
    NesError(#[from] NesError),
    
    #[error("Invalid iNES header")]
    InvalidHeader,
    
    #[error("Unsupported mapper: {0}")]
    UnsupportedMapper(u8),
    
    #[error("Invalid ROM format")]
    InvalidFormat,
}

/// Extract the first NES ROM file from a ZIP archive
fn extract_nes_rom_from_zip(zip_path: &Path) -> Result<Vec<u8>, RomError> {
    let file = File::open(zip_path)?;
    let mut archive = ZipArchive::new(file)?;
    
    // Look for the first .nes file in the archive
    for i in 0..archive.len() {
        let mut file = archive.by_index(i)?;
        let outpath = file.name().to_lowercase();
        
        if outpath.ends_with(".nes") {
            let mut buffer = Vec::new();
            file.read_to_end(&mut buffer)?;
            return Ok(buffer);
        }
    }
    
    Err(RomError::InvalidFormat)
}

/// Load a ROM file into the NES
pub fn load_rom<P: AsRef<Path>>(nes: &mut Nes, path: P) -> Result<(), RomError> {
    let path = path.as_ref();
    
    // Extract ROM data from file or ZIP
    let rom_data = if let Some(ext) = path.extension().and_then(|s| s.to_str()) {
        match ext.to_lowercase().as_str() {
            "zip" => extract_nes_rom_from_zip(path)?,
            "nes" => {
                let mut file = File::open(path)?;
                let mut buffer = Vec::new();
                file.read_to_end(&mut buffer)?;
                buffer
            }
            _ => return Err(RomError::InvalidFormat),
        }
    } else {
        return Err(RomError::InvalidFormat);
    };
    
    // Save ROM data to a temporary file and pass the path to nes.load_rom
    let temp_dir = std::env::temp_dir();
    let temp_path = temp_dir.join("temp_rom.nes");
    std::fs::write(&temp_path, &rom_data)?;
    
    // Load the ROM into the NES
    nes.load_rom(&temp_path)?;
    
    // Clean up the temporary file
    let _ = std::fs::remove_file(temp_path);
    
    Ok(())
}

/// Check if a file is a valid ROM file (supports .nes and .zip)
fn is_rom_file(entry: &std::fs::DirEntry) -> bool {
    if let Some(ext) = entry.path().extension().and_then(|s| s.to_str()) {
        matches!(ext.to_lowercase().as_str(), "nes" | "zip")
    } else {
        false
    }
}

/// List available ROMs in a directory
pub fn list_roms<P: AsRef<Path>>(dir: P) -> Result<Vec<String>, std::io::Error> {
    let mut roms = Vec::new();
    
    for entry in std::fs::read_dir(dir)? {
        let entry = entry?;
        if !entry.file_type()?.is_file() {
            continue;
        }
        
        if is_rom_file(&entry) {
            if let Some(file_name) = entry.file_name().to_str() {
                roms.push(file_name.to_string());
            }
        }
    }
    
    roms.sort();
    Ok(roms)
}
