//! NES Mapper implementations
//!
//! This module provides implementations of various NES mappers that handle
//! memory mapping and bank switching for different cartridge types.

// Re-export mappers
pub mod mmc2;
pub mod mmc3;

// Import mappers
use mmc3::MMC3;

use std::cell::RefCell;
use std::fmt;
use std::rc::Rc;

/// Trait for NES mappers
pub trait Mapper: fmt::Debug + Send + Sync {
    /// Read a byte from the specified address
    fn read(&self, addr: u16) -> u8;

    /// Write a byte to the specified address
    fn write(&mut self, addr: u16, value: u8);

    /// Read a byte from PRG ROM/RAM
    fn read_prg_byte(&self, addr: u16) -> u8 {
        self.read(addr)
    }

    /// Write a byte to PRG ROM/RAM
    fn write_prg_byte(&mut self, addr: u16, value: u8) {
        self.write(addr, value);
    }

    /// Get a reference to the PRG ROM
    fn prg_rom(&self) -> &[u8];

    /// Get a mutable reference to the PRG ROM
    fn prg_rom_mut(&mut self) -> &mut [u8];

    /// Get a reference to the CHR ROM (if any)
    fn chr_rom(&self) -> Option<&[u8]>;

    /// Get a mutable reference to the CHR ROM (if any)
    fn chr_rom_mut(&mut self) -> Option<&mut [u8]>;

    /// Get the mirroring mode
    fn mirroring(&self) -> Mirroring;

    /// Reset the mapper to its initial state
    fn reset(&mut self);
}

/// Mirroring modes for nametables
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mirroring {
    /// Horizontal mirroring (vertical arrangement)
    Horizontal,

    /// Vertical mirroring (horizontal arrangement)
    Vertical,

    /// Four-screen VRAM (used by some mappers)
    FourScreen,

    /// Single-screen mirroring (all nametables mirror the same 1KB of VRAM)
    SingleScreen(u8),
}

/// Create a mapper based on the mapper number and ROM data
pub fn create_mapper(
    mapper_num: u8,
    prg_rom: Vec<u8>,
    chr_rom: Option<Vec<u8>>,
    mirroring: Mirroring,
) -> Result<Rc<RefCell<dyn Mapper>>, crate::nes::NesError> {
    match mapper_num {
        // NROM (Mapper 0)
        0 => Ok(Rc::new(RefCell::new(Nrom::new(prg_rom, chr_rom, mirroring)))),

        // MMC1 (Mapper 1)
        1 => Ok(Rc::new(RefCell::new(Mmc1::new(prg_rom, chr_rom, mirroring)))),

        // MMC3 (Mapper 4)
        4 => Ok(Rc::new(RefCell::new(MMC3::new(prg_rom, chr_rom, mirroring)))),

        // Unsupported mapper
        n => Err(crate::nes::NesError::UnsupportedMapper(n)),
    }
}

/// NROM (Mapper 0)
///
/// The simplest mapper with no bank switching. It comes in two variants:
/// - NROM-128: 16KB PRG ROM, 8KB CHR ROM
/// - NROM-256: 32KB PRG ROM, 8KB CHR ROM
#[derive(Debug)]
pub struct Nrom {
    prg_rom: Vec<u8>,
    chr_rom: Option<Vec<u8>>,
    mirroring: Mirroring,
}

impl Nrom {
    /// Create a new NROM mapper
    pub fn new(prg_rom: Vec<u8>, chr_rom: Option<Vec<u8>>, mirroring: Mirroring) -> Self {
        Nrom {
            prg_rom,
            chr_rom,
            mirroring,
        }
    }
}

impl Mapper for Nrom {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            // CHR ROM/RAM (0x0000-0x1FFF)
            0x0000..=0x1FFF => {
                if let Some(chr) = &self.chr_rom {
                    chr[addr as usize]
                } else {
                    0 // CHR RAM not implemented
                }
            }

            // PRG ROM (0x8000-0xFFFF)
            0x8000..=0xFFFF => {
                let addr = (addr - 0x8000) as usize % self.prg_rom.len();
                self.prg_rom[addr]
            }

            _ => 0,
        }
    }

    fn write(&mut self, addr: u16, value: u8) {
        // CHR RAM writes
        if addr < 0x2000 && self.chr_rom.is_none() {
            // If CHR RAM, write to it, but not implemented
            let _ = (addr, value); // Suppress unused variable warnings
        }
    }

    fn prg_rom(&self) -> &[u8] {
        &self.prg_rom
    }

    fn prg_rom_mut(&mut self) -> &mut [u8] {
        &mut self.prg_rom
    }

    fn chr_rom(&self) -> Option<&[u8]> {
        self.chr_rom.as_deref()
    }

    fn chr_rom_mut(&mut self) -> Option<&mut [u8]> {
        self.chr_rom.as_deref_mut()
    }

    fn mirroring(&self) -> Mirroring {
        self.mirroring
    }

    fn reset(&mut self) {
        // NROM has no state to reset
    }
}

/// MMC1 (Mapper 1)
///
/// The MMC1 is a versatile mapper with up to 512KB PRG ROM and 256KB CHR ROM.
/// It supports bank switching for both PRG and CHR ROM, as well as multiple
/// mirroring modes.
#[derive(Debug)]
pub struct Mmc1 {
    prg_rom: Vec<u8>,
    chr_rom: Option<Vec<u8>>,
    shift_register: u8,
    shift_count: u8,
    control: u8,
    prg_bank: u8,
    chr_bank0: u8,
    chr_bank1: u8,
    prg_ram: [u8; 0x2000],
    mirroring: Mirroring,
}

impl Mmc1 {
    /// Create a new MMC1 mapper
    pub fn new(prg_rom: Vec<u8>, chr_rom: Option<Vec<u8>>, mirroring: Mirroring) -> Self {
        Mmc1 {
            prg_rom,
            chr_rom,
            shift_register: 0,
            shift_count: 0,
            control: 0x0C,
            prg_bank: 0,
            chr_bank0: 0,
            chr_bank1: 0,
            prg_ram: [0; 0x2000],
            mirroring,
        }
    }

    fn update_mirroring(&mut self) {
        match self.control & 0x03 {
            0 => self.mirroring = Mirroring::SingleScreen(0),
            1 => self.mirroring = Mirroring::SingleScreen(1),
            2 => self.mirroring = Mirroring::Vertical,
            3 => self.mirroring = Mirroring::Horizontal,
            _ => unreachable!(),
        }
    }

    fn prg_bank_mode(&self) -> u8 {
        (self.control >> 2) & 0x03
    }

    fn chr_bank_mode(&self) -> bool {
        (self.control & 0x10) != 0
    }
}

impl Mapper for Mmc1 {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            // CHR ROM (0x0000-0x1FFF)
            0x0000..=0x1FFF => {
                if let Some(chr) = &self.chr_rom {
                    let bank_size = if self.chr_bank_mode() { 0x1000 } else { 0x2000 };
                    let bank = if addr < 0x1000 {
                        self.chr_bank0 as usize
                    } else {
                        self.chr_bank1 as usize
                    };
                    let addr_offset = (addr as usize) % bank_size;
                    let chr_addr = bank * bank_size + addr_offset;
                    if chr_addr < chr.len() {
                        chr[chr_addr]
                    } else {
                        0
                    }
                } else {
                    0
                }
            }

            // PRG RAM (0x6000-0x7FFF)
            0x6000..=0x7FFF => self.prg_ram[(addr - 0x6000) as usize],

            // PRG ROM (0x8000-0xFFFF)
            0x8000..=0xFFFF => {
                let bank_mode = self.prg_bank_mode();
                let (bank, addr_offset) = match (bank_mode, addr) {
                    // 32KB mode (ignores bit 0 of bank number)
                    (0, _) | (1, _) => {
                        let bank = (self.prg_bank & 0x0E) as usize;
                        let addr_offset = (addr - 0x8000) as usize;
                        (bank, addr_offset)
                    }

                    // Fix first bank at $8000, switch 16KB bank at $C000
                    (2, 0x8000..=0xBFFF) => (0, (addr - 0x8000) as usize),
                    (2, 0xC000..=0xFFFF) => {
                        let bank = (self.prg_bank & 0x0F) as usize;
                        (bank, (addr - 0xC000) as usize)
                    }

                    // Fix last bank at $C000, switch 16KB bank at $8000
                    (3, 0x8000..=0xBFFF) => {
                        let bank = (self.prg_bank & 0x0F) as usize;
                        (bank, (addr - 0x8000) as usize)
                    }
                    (3, 0xC000..=0xFFFF) => {
                        let last_bank = (self.prg_rom.len() / 0x4000 - 1) as usize;
                        (last_bank, (addr - 0xC000) as usize)
                    }

                    _ => unreachable!(),
                };

                let addr = bank * 0x4000 + addr_offset;
                if addr < self.prg_rom.len() {
                    self.prg_rom[addr]
                } else {
                    0
                }
            }

            _ => 0,
        }
    }

    fn write(&mut self, addr: u16, value: u8) {
        match addr {
            // MMC1 registers (0x8000-0xFFFF)
            0x8000..=0xFFFF => {
                // If bit 7 is set, reset the shift register
                if (value & 0x80) != 0 {
                    self.shift_register = 0;
                    self.shift_count = 0;
                    self.control |= 0x0C; // Set PRG ROM bank mode to 3
                    return;
                }

                // Shift in the data bit
                self.shift_register >>= 1;
                self.shift_register |= (value & 0x01) << 4;
                self.shift_count += 1;

                // If we've shifted in 5 bits, write to the appropriate register
                if self.shift_count == 5 {
                    let reg_addr = (addr >> 13) & 0x03;

                    match reg_addr {
                        // Control register (0x8000-0x9FFF)
                        0 => {
                            self.control = self.shift_register & 0x1F;
                            self.update_mirroring();
                        }

                        // CHR bank 0 (0xA000-0xBFFF)
                        1 => {
                            self.chr_bank0 = self.shift_register & 0x1F;
                        }

                        // CHR bank 1 (0xC000-0xDFFF)
                        2 => {
                            self.chr_bank1 = self.shift_register & 0x1F;
                        }

                        // PRG bank (0xE000-0xFFFF)
                        3 => {
                            self.prg_bank = self.shift_register & 0x0F;
                        }

                        _ => unreachable!(),
                    }

                    // Reset the shift register
                    self.shift_register = 0;
                    self.shift_count = 0;
                }
            }

            _ => {}
        }
    }

    fn prg_rom(&self) -> &[u8] {
        &self.prg_rom
    }

    fn prg_rom_mut(&mut self) -> &mut [u8] {
        &mut self.prg_rom
    }

    fn chr_rom(&self) -> Option<&[u8]> {
        self.chr_rom.as_deref()
    }

    fn chr_rom_mut(&mut self) -> Option<&mut [u8]> {
        self.chr_rom.as_deref_mut()
    }

    fn mirroring(&self) -> Mirroring {
        self.mirroring
    }

    fn reset(&mut self) {
        self.shift_register = 0;
        self.shift_count = 0;
        self.control = 0x0C;
        self.prg_bank = 0;
        self.chr_bank0 = 0;
        self.chr_bank1 = 0;
        self.update_mirroring();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nrom_mapper() {
        // Create a simple NROM-128 ROM (16KB PRG ROM, 8KB CHR ROM)
        let prg_rom = vec![0; 0x4000];
        let chr_rom = Some(vec![0; 0x2000]);
        let mirroring = Mirroring::Horizontal;

        let mut mapper = Nrom::new(prg_rom, chr_rom, mirroring);

        // Test mirroring
        assert_eq!(mapper.mirroring(), Mirroring::Horizontal);

        // Test PRG ROM access (should mirror every 16KB)
        mapper.prg_rom_mut()[0] = 0x42;
        assert_eq!(mapper.read(0x8000), 0x42);
        assert_eq!(mapper.read(0xC000), 0x42);

        // Test CHR ROM access
        if let Some(chr_rom) = mapper.chr_rom_mut() {
            chr_rom[0] = 0x84;
            assert_eq!(chr_rom[0], 0x84);
        }
    }

    #[test]
    fn test_mmc1_mapper() {
        // Create a simple MMC1 ROM (32KB PRG ROM, 8KB CHR ROM)
        let prg_rom = vec![0; 0x8000];
        let chr_rom = Some(vec![0; 0x2000]);
        let mirroring = Mirroring::Vertical;

        let mut mapper = Mmc1::new(prg_rom, chr_rom, mirroring);

        // Test initial state
        assert_eq!(mapper.mirroring(), Mirroring::Horizontal); // Default after reset

        // Test control register write
        mapper.write(0x8000, 0x88); // Reset shift register
        mapper.write(0x8000, 0x03); // Write 0x03 to control register (Horizontal mirroring)

        // Test PRG bank switching
        mapper.write(0xE000, 0x88); // Reset shift register
        mapper.write(0xE000, 0x01); // Switch to PRG bank 1

        // Test CHR bank switching
        mapper.write(0xA000, 0x88); // Reset shift register
        mapper.write(0xA000, 0x02); // Switch to CHR bank 2
    }
}
