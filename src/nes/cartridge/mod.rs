//! NES Cartridge handling

use std::fs::File;
use std::io::{self, Read};
use std::path::Path;

/// Represents an NES cartridge
#[derive(Debug, Clone)]
pub struct Cartridge {
    /// PRG ROM data (program data)
    pub prg_rom: Vec<u8>,
    
    /// CHR ROM data (pattern tables)
    pub chr_rom: Vec<u8>,
    
    /// Mapper number
    pub mapper: u8,
    
    /// Mirroring type
    pub mirroring: super::mapper::Mirroring,
    
    /// Whether the cartridge has battery-backed RAM
    pub has_battery: bool,
    
    /// Whether the cartridge has trainer data
    pub has_trainer: bool,
    
    /// Whether the cartridge has PRG RAM
    pub has_prg_ram: bool,
    
    /// Whether the cartridge is for VS Unisystem
    pub is_vs_unisystem: bool,
    
    /// Whether the cartridge is for PlayChoice-10
    pub is_playchoice: bool,
}

impl Cartridge {
    /// Create a new Cartridge by loading a ROM file
    pub fn from_file<P: AsRef<Path>>(path: P) -> io::Result<Self> {
        let mut file = File::open(path)?;
        let mut header = [0u8; 16];
        file.read_exact(&mut header)?;
        
        // Check for iNES magic number
        if &header[0..4] != b"NES\x1A" {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "Not a valid iNES file",
            ));
        }
        
        let prg_rom_size = header[4] as usize * 16 * 1024; // 16KB units
        let chr_rom_size = header[5] as usize * 8 * 1024;   // 8KB units
        
        let flags6 = header[6];
        let flags7 = header[7];
        
        let mapper_lower = (flags6 & 0xF0) >> 4;
        let mapper_upper = flags7 & 0xF0;
        let mapper = mapper_upper | mapper_lower;
        
        let mirroring = if (flags6 & 0x08) != 0 {
            super::mapper::Mirroring::FourScreen
        } else if (flags6 & 0x01) != 0 {
            super::mapper::Mirroring::Vertical
        } else {
            super::mapper::Mirroring::Horizontal
        };
        
        let has_battery = (flags6 & 0x02) != 0;
        let has_trainer = (flags6 & 0x04) != 0;
        let has_prg_ram = (flags6 & 0x02) != 0;
        let is_vs_unisystem = (flags7 & 0x01) != 0;
        let is_playchoice = (flags7 & 0x02) != 0;
        
        // Skip trainer if present
        if has_trainer {
            file.seek(std::io::SeekFrom::Current(512))?;
        }
        
        // Read PRG ROM
        let mut prg_rom = vec![0u8; prg_rom_size];
        file.read_exact(&mut prg_rom)?;
        
        // Read CHR ROM (or create CHR RAM if size is 0)
        let mut chr_rom = if chr_rom_size > 0 {
            let mut rom = vec![0u8; chr_rom_size];
            file.read_exact(&mut rom)?;
            rom
        } else {
            // 8KB of CHR RAM
            vec![0u8; 8 * 1024]
        };
        
        Ok(Self {
            prg_rom,
            chr_rom,
            mapper,
            mirroring,
            has_battery,
            has_trainer,
            has_prg_ram,
            is_vs_unisystem,
            is_playchoice,
        })
    }
    
    /// Create a new Cartridge with the given ROM data
    pub fn new(prg_rom: Vec<u8>, chr_rom: Vec<u8>, mapper: u8, mirroring: super::mapper::Mirroring) -> Self {
        Self {
            prg_rom,
            chr_rom,
            mapper,
            mirroring,
            has_battery: false,
            has_trainer: false,
            has_prg_ram: false,
            is_vs_unisystem: false,
            is_playchoice: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;
    
    fn create_test_rom() -> io::Result<NamedTempFile> {
        let mut file = NamedTempFile::new()?;
        
        // iNES header
        file.write_all(b"NES\x1A")?;  // Magic number
        file.write_all(&[0x02, 0x01])?;  // 32KB PRG ROM, 8KB CHR ROM
        file.write_all(&[0x00, 0x00])?;  // Flags 6 & 7 (mapper 0, horizontal mirroring)
        file.write_all(&[0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])?;  // Padding
        
        // Dummy PRG ROM (32KB)
        file.write_all(&vec![0x00; 32 * 1024])?;
        
        // Dummy CHR ROM (8KB)
        file.write_all(&vec![0x00; 8 * 1024])?;
        
        Ok(file)
    }
    
    #[test]
    fn test_load_rom() {
        let mut test_rom = create_test_rom().unwrap();
        let path = test_rom.path();
        
        let cart = Cartridge::from_file(path).unwrap();
        
        assert_eq!(cart.prg_rom.len(), 32 * 1024);
        assert_eq!(cart.chr_rom.len(), 8 * 1024);
        assert_eq!(cart.mapper, 0);
        assert_eq!(cart.mirroring, super::super::mapper::Mirroring::Horizontal);
    }
    
    #[test]
    fn test_invalid_rom() {
        let mut test_rom = NamedTempFile::new().unwrap();
        test_rom.write_all(b"INVALID").unwrap();
        
        let result = Cartridge::from_file(test_rom.path());
        assert!(result.is_err());
    }
}
