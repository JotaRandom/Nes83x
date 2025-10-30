//! iNES Mapper 9 (MMC2) implementation
//! 
//! The MMC2 is used by a few games including Punch-Out!! and Mike Tyson's Punch-Out!!

use crate::nes::mapper::{Mapper, Mirroring};
use crate::nes::cartridge::Cartridge;

/// Mapper 9: MMC2 (PxROM)
///
/// The MMC2 is a mapper chip used by Nintendo for a small number of games.
/// It features:
/// - 32KB PRG ROM (fixed to first and last 16KB banks)
/// - 8KB CHR ROM with 4KB bank switching
/// - 1KB of PRG RAM (not battery-backed)
/// - Latch-based CHR bank switching
#[derive(Debug)]
pub struct MMC2 {
    prg_rom: Vec<u8>,
    chr_rom: Vec<u8>,
    prg_ram: [u8; 0x1000],
    
    // Mapper registers
    prg_bank: u8,
    chr_bank0: u8,
    chr_bank1: u8,
    mirroring: Mirroring,
    
    // Latch registers
    latch0: u8,
    latch1: u8,
    
    // Current CHR banks (determined by latches)
    current_chr_bank0: u8,
    current_chr_bank1: u8,
}

impl MMC2 {
    /// Create a new MMC2 mapper from a cartridge
    pub fn new(cartridge: &Cartridge) -> Self {
        MMC2 {
            prg_rom: cartridge.prg_rom.clone(),
            chr_rom: cartridge.chr_rom.clone(),
            prg_ram: [0; 0x1000],
            prg_bank: 0,
            chr_bank0: 0,
            chr_bank1: 0,
            mirroring: cartridge.mirroring,
            latch0: 0xFE, // Default to $FD area
            latch1: 0xFE, // Default to $FD area
            current_chr_bank0: 0,
            current_chr_bank1: 0,
        }
    }
    
    /// Update the current CHR banks based on latches
    fn update_chr_banks(&mut self) {
        if self.latch0 & 0x01 == 0 {
            self.current_chr_bank0 = self.chr_bank0 & 0x1F;
        } else {
            self.current_chr_bank0 = (self.chr_bank0 >> 4) & 0x1F;
        }
        
        if self.latch1 & 0x01 == 0 {
            self.current_chr_bank1 = self.chr_bank1 & 0x1F;
        } else {
            self.current_chr_bank1 = (self.chr_bank1 >> 4) & 0x1F;
        }
    }
    
    /// Check if an address is in the latch 0 range ($0FD8-$0FDF)
    fn is_latch0_address(addr: u16) -> bool {
        (addr & 0x1FF8) == 0x0FD8
    }
    
    /// Check if an address is in the latch 1 range ($0FE8-$0FEF)
    fn is_latch1_address(addr: u16) -> bool {
        (addr & 0x1FF8) == 0x0FE8
    }
    
    /// Update latches based on PPU address
    pub fn update_latches(&mut self, addr: u16) {
        if Self::is_latch0_address(addr) {
            self.latch0 = 0xFD; // Switch to $FD area
            self.update_chr_banks();
        } else if Self::is_latch1_address(addr) {
            self.latch1 = 0xFD; // Switch to $FD area
            self.update_chr_banks();
        } else if (addr & 0x1000) != 0 {
            // Switch back to $FE area if accessing pattern table 1
            if (self.latch0 & 0xFE) == 0xFD {
                self.latch0 = 0xFE;
                self.update_chr_banks();
            }
            if (self.latch1 & 0xFE) == 0xFD {
                self.latch1 = 0xFE;
                self.update_chr_banks();
            }
        }
    }
}

impl Mapper for MMC2 {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            // PRG ROM (0x8000-0xFFFF)
            0x8000..=0xBFFF => {
                // First 16KB bank
                let bank = 0;
                let offset = (addr - 0x8000) as usize + (bank * 0x4000) % self.prg_rom.len();
                self.prg_rom[offset]
            }
            0xC000..=0xFFFF => {
                // Last 16KB bank (fixed)
                let bank = (self.prg_rom.len() / 0x4000).saturating_sub(1);
                let offset = (addr - 0xC000) as usize + (bank * 0x4000) % self.prg_rom.len();
                self.prg_rom[offset]
            }
            // PRG RAM (0x6000-0x7FFF)
            0x6000..=0x7FFF => {
                self.prg_ram[(addr - 0x6000) as usize]
            }
            _ => 0xFF,
        }
    }
    
    fn write(&mut self, addr: u16, value: u8) {
        match addr {
            // PRG RAM (0x6000-0x7FFF)
            0x6000..=0x7FFF => {
                self.prg_ram[(addr - 0x6000) as usize] = value;
            }
            // Bank select ($A000-$AFFF)
            0xA000..=0xAFFF => {
                self.prg_bank = value & 0x0F;
            }
            // CHR bank 0 ($B000-$BFFF)
            0xB000..=0xBFFF => {
                self.chr_bank0 = value;
                self.update_chr_banks();
            }
            // CHR bank 1 ($C000-$CFFF)
            0xC000..=0xCFFF => {
                self.chr_bank1 = value;
                self.update_chr_banks();
            }
            // Mirroring ($D000-$DFFF)
            0xD000..=0xDFFF => {
                self.mirroring = if (value & 0x01) != 0 {
                    Mirroring::Horizontal
                } else {
                    Mirroring::Vertical
                };
            }
            // PRG RAM protect ($E000-$FFFF)
            0xE000..=0xFFFF => {
                // PRG RAM write protection is not implemented
                // as most emulators ignore it
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
        Some(&self.chr_rom)
    }
    
    fn chr_rom_mut(&mut self) -> Option<&mut [u8]> {
        Some(&mut self.chr_rom)
    }
    
    fn mirroring(&self) -> Mirroring {
        self.mirroring
    }
    
    fn reset(&mut self) {
        self.prg_bank = 0;
        self.chr_bank0 = 0;
        self.chr_bank1 = 0;
        self.latch0 = 0xFE;
        self.latch1 = 0xFE;
        self.update_chr_banks();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nes::cartridge::Cartridge;
    
    fn create_test_cartridge() -> Cartridge {
        // Create a test cartridge with 32KB PRG-ROM and 8KB CHR-ROM
        let mut prg_rom = vec![0; 0x8000];
        let chr_rom = vec![0; 0x2000];
        
        // Fill with test patterns
        for (i, byte) in prg_rom.iter_mut().enumerate() {
            *byte = (i & 0xFF) as u8;
        }
        
        Cartridge {
            prg_rom,
            chr_rom,
            mapper: 9,
            mirroring: Mirroring::Horizontal,
            has_battery: false,
            has_trainer: false,
            has_prg_ram: false,
            is_vs_unisystem: false,
            is_playchoice: false,
        }
    }
    
    #[test]
    fn test_mmc2_initial_state() {
        let cartridge = create_test_cartridge();
        let mmc2 = MMC2::new(&cartridge);
        
        assert_eq!(mmc2.prg_bank, 0);
        assert_eq!(mmc2.chr_bank0, 0);
        assert_eq!(mmc2.chr_bank1, 0);
        assert_eq!(mmc2.mirroring, Mirroring::Horizontal);
        assert_eq!(mmc2.latch0, 0xFE);
        assert_eq!(mmc2.latch1, 0xFE);
    }
    
    #[test]
    fn test_mmc2_prg_banking() {
        let cartridge = create_test_cartridge();
        let mut mmc2 = MMC2::new(&cartridge);
        
        // Write to PRG bank register
        mmc2.write(0xA000, 0x05);
        assert_eq!(mmc2.prg_bank, 0x05);
        
        // First 16KB bank should still be bank 0 (fixed)
        assert_eq!(mmc2.read(0x8000), 0x00u8);
        
        // Last 16KB bank should be the last bank
        let last_bank = (mmc2.prg_rom.len() / 0x4000).saturating_sub(1);
        assert_eq!(mmc2.read(0xC000 + (last_bank * 0x4000)), 0x00u8);
    }
    
    #[test]
    fn test_mmc2_chr_banking() {
        let cartridge = create_test_cartridge();
        let mut mmc2 = MMC2::new(&cartridge);
        
        // Write to CHR bank 0
        mmc2.write(0xB000, 0x12);
        assert_eq!(mmc2.chr_bank0, 0x12);
        
        // Write to CHR bank 1
        mmc2.write(0xC000, 0x34);
        assert_eq!(mmc2.chr_bank1, 0x34);
    }
    
    #[test]
    fn test_mmc2_mirroring() {
        let cartridge = create_test_cartridge();
        let mut mmc2 = MMC2::new(&cartridge);
        
        // Test horizontal mirroring
        mmc2.write(0xD000, 0x01);
        assert_eq!(mmc2.mirroring(), Mirroring::Horizontal);
        
        // Test vertical mirroring
        mmc2.write(0xD000, 0x00);
        assert_eq!(mmc2.mirroring(), Mirroring::Vertical);
    }
    
    #[test]
    fn test_mmc2_latches() {
        let cartridge = create_test_cartridge();
        let mut mmc2 = MMC2::new(&cartridge);
        
        // Set up CHR banks
        mmc2.write(0xB000, 0x12); // CHR bank 0: $FD=0x12, $FE=0x01
        mmc2.write(0xC000, 0x34); // CHR bank 1: $FD=0x34, $FE=0x03
        
        // Initial state should be $FE area
        assert_eq!(mmc2.latch0, 0xFE);
        assert_eq!(mmc2.latch1, 0xFE);
        assert_eq!(mmc2.current_chr_bank0, 0x01);
        assert_eq!(mmc2.current_chr_bank1, 0x03);
        
        // Trigger latch 0
        mmc2.update_latches(0x0FD8);
        assert_eq!(mmc2.latch0, 0xFD);
        assert_eq!(mmc2.current_chr_bank0, 0x12);
        
        // Trigger latch 1
        mmc2.update_latches(0x0FE8);
        assert_eq!(mmc2.latch1, 0xFD);
        assert_eq!(mmc2.current_chr_bank1, 0x34);
        
        // Switch back to $FE area
        mmc2.update_latches(0x1000);
        assert_eq!(mmc2.latch0, 0xFE);
        assert_eq!(mmc2.latch1, 0xFE);
        assert_eq!(mmc2.current_chr_bank0, 0x01);
        assert_eq!(mmc2.current_chr_bank1, 0x03);
    }
}
