use std::cell::RefCell;
use std::rc::Rc;

/// Trait for mappers that handle bank switching and memory mapping
pub trait Mapper: std::fmt::Debug {
    /// Read a byte from the specified address
    fn read(&self, addr: u16) -> u8;
    
    /// Write a byte to the specified address
    fn write(&mut self, addr: u16, value: u8);
    
    /// Get a reference to the CHR ROM (if available)
    fn chr_rom(&self) -> Option<&[u8]>;
    
    /// Get a mutable reference to the CHR ROM (if available)
    fn chr_rom_mut(&mut self) -> Option<&mut [u8]>;
}

/// Represents the PPU's memory (VRAM, palette RAM, etc.)
#[derive(Debug)]
pub struct PpuMemory {
    /// Pattern tables (CHR ROM/RAM)
    pattern_tables: [[u8; 0x1000]; 2],
    
    /// Nametables (VRAM)
    nametables: [[u8; 0x0400]; 4],
    
    /// Palette RAM
    palette_ram: [u8; 0x20],
    
    /// Mirroring mode for nametables
    mirroring: Mirroring,
    
    /// Mapper for bank switching
    mapper: Option<Rc<RefCell<dyn Mapper>>>,
}

/// Nametable mirroring modes
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mirroring {
    /// Horizontal mirroring (vertical arrangement)
    Horizontal,
    
    /// Vertical mirroring (horizontal arrangement)
    Vertical,
    
    /// Single-screen mirroring (all nametables mirror the same 1KB of VRAM)
    SingleScreen(u8),
    
    /// Four-screen VRAM (rare, used by some mappers)
    FourScreen,
}

impl Default for Mirroring {
    fn default() -> Self {
        Mirroring::Horizontal
    }
}

impl PpuMemory {
    /// Create a new PPU memory instance
    pub fn new() -> Self {
        PpuMemory {
            pattern_tables: [[0; 0x1000]; 2],
            nametables: [[0; 0x0400]; 4],
            palette_ram: [0; 0x20],
            mirroring: Mirroring::default(),
            mapper: None,
        }
    }
    
    /// Set the mapper for bank switching
    pub fn set_mapper(&mut self, mapper: Rc<RefCell<dyn Mapper>>) {
        self.mapper = Some(mapper);
    }
    
    /// Set the mirroring mode
    pub fn set_mirroring(&mut self, mirroring: Mirroring) {
        self.mirroring = mirroring;
    }
    
    /// Read a byte from the specified address
    pub fn read(&self, addr: u16) -> u8 {
        match addr {
            // Pattern tables (0x0000-0x1FFF)
            0x0000..=0x1FFF => {
                let table = (addr >> 12) as usize & 0x1;
                let offset = addr & 0x0FFF;
                self.pattern_tables[table][offset as usize]
            }
            
            // Nametables (0x2000-0x3EFF)
            0x2000..=0x3EFF => {
                let addr = self.nametable_mirror(addr);
                let table = ((addr - 0x2000) >> 10) as usize & 0x3;
                let offset = (addr - 0x2000) & 0x3FF;
                self.nametables[table][offset as usize]
            }
            
            // Palette RAM (0x3F00-0x3FFF)
            0x3F00..=0x3FFF => {
                let addr = addr & 0x1F;
                // Handle mirroring of addresses 0x3F10, 0x3F14, 0x3F18, 0x3F1C
                let addr = if addr & 0x03 == 0 { addr & 0x0F } else { addr };
                self.palette_ram[addr as usize]
            }
            
            _ => 0,
        }
    }
    
    /// Write a byte to the specified address
    pub fn write(&mut self, addr: u16, value: u8) {
        match addr {
            // Pattern tables (0x0000-0x1FFF)
            0x0000..=0x1FFF => {
                if let Some(mapper) = &mut self.mapper {
                    if let Some(chr_rom) = mapper.borrow_mut().chr_rom_mut() {
                        // If we have a mapper with CHR RAM, write to it
                        chr_rom[addr as usize] = value;
                    }
                } else {
                    // No mapper, write to internal pattern tables
                    let table = (addr >> 12) as usize & 0x1;
                    let offset = addr & 0x0FFF;
                    self.pattern_tables[table][offset as usize] = value;
                }
            }
            
            // Nametables (0x2000-0x3EFF)
            0x2000..=0x3EFF => {
                let addr = self.nametable_mirror(addr);
                let table = ((addr - 0x2000) >> 10) as usize & 0x3;
                let offset = (addr - 0x2000) & 0x3FF;
                self.nametables[table][offset as usize] = value;
            }
            
            // Palette RAM (0x3F00-0x3FFF)
            0x3F00..=0x3FFF => {
                let mut addr = addr & 0x1F;
                // Handle mirroring of addresses 0x3F10, 0x3F14, 0x3F18, 0x3F1C
                if addr & 0x03 == 0 {
                    addr &= 0x0F;
                }
                self.palette_ram[addr as usize] = value;
            }
            
            _ => {}
        }
    }
    
    /// Handle nametable mirroring
    fn nametable_mirror(&self, addr: u16) -> u16 {
        let addr = (addr - 0x2000) % 0x1000;
        let table = addr / 0x0400;
        let offset = addr % 0x0400;
        
        match self.mirroring {
            Mirroring::Horizontal => {
                // Horizontal: $2000 equals $2400 and $2800 equals $2C00
                let mirrored_table = table & 0x1;
                0x2000 + (mirrored_table * 0x0400) as u16 + offset
            }
            Mirroring::Vertical => {
                // Vertical: $2000 equals $2800 and $2400 equals $2C00
                let mirrored_table = table % 2;
                0x2000 + (mirrored_table * 0x0400) as u16 + offset
            }
            Mirroring::SingleScreen(n) => {
                // Single-screen: all nametables mirror the same 1KB of VRAM
                0x2000 + (n as u16 * 0x0400) + offset
            }
            Mirroring::FourScreen => {
                // Four-screen: all 4 nametables are separate (no mirroring)
                0x2000 + addr
            }
        }
    }
    
    /// Read a 16-bit word from the specified address (little-endian)
    pub fn read_word(&self, addr: u16) -> u16 {
        let lo = self.read(addr) as u16;
        let hi = self.read(addr.wrapping_add(1)) as u16;
        (hi << 8) | lo
    }
    
    /// Write a 16-bit word to the specified address (little-endian)
    pub fn write_word(&mut self, addr: u16, value: u16) {
        self.write(addr, value as u8);
        self.write(addr.wrapping_add(1), (value >> 8) as u8);
    }
    
    /// Read a byte from the pattern table
    pub fn read_pattern_byte(&self, table: u8, tile: u8, row: u8, plane: u8) -> u8 {
        let addr = (table as u16 * 0x1000) + (tile as u16 * 16) + (row as u16) + (plane as u16 * 8);
        self.read(addr)
    }
    
    /// Read a color from the palette
    pub fn read_palette(&self, addr: u8) -> u8 {
        let addr = (addr as u16) & 0x1F;
        self.read(0x3F00 | addr)
    }
    
    /// Write a color to the palette
    pub fn write_palette(&mut self, addr: u8, value: u8) {
        let addr = (addr as u16) & 0x1F;
        self.write(0x3F00 | addr, value);
    }
    
    /// Get a reference to the palette RAM
    pub fn palette_ram(&self) -> &[u8; 0x20] {
        &self.palette_ram
    }
    
    /// Get a reference to the pattern tables
    pub fn pattern_tables(&self) -> &[[u8; 0x1000]; 2] {
        &self.pattern_tables
    }
    
    /// Get a reference to the nametables
    pub fn nametables(&self) -> &[[u8; 0x0400]; 4] {
        &self.nametables
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_nametable_mirroring_horizontal() {
        let mut ppu_mem = PpuMemory::new();
        ppu_mem.set_mirroring(Mirroring::Horizontal);
        
        // Write to 0x2000 and 0x2400
        ppu_mem.write(0x2000, 0x42);
        ppu_mem.write(0x2400, 0x84);
        
        // 0x2000 and 0x2400 should be different
        assert_ne!(ppu_mem.read(0x2000), ppu_mem.read(0x2400));
        
        // 0x2000 and 0x2800 should be the same (horizontal mirroring)
        assert_eq!(ppu_mem.read(0x2000), ppu_mem.read(0x2800));
        
        // 0x2400 and 0x2C00 should be the same (horizontal mirroring)
        assert_eq!(ppu_mem.read(0x2400), ppu_mem.read(0x2C00));
    }
    
    #[test]
    fn test_nametable_mirroring_vertical() {
        let mut ppu_mem = PpuMemory::new();
        ppu_mem.set_mirroring(Mirroring::Vertical);
        
        // Write to 0x2000 and 0x2800
        ppu_mem.write(0x2000, 0x42);
        ppu_mem.write(0x2800, 0x84);
        
        // 0x2000 and 0x2800 should be different
        assert_ne!(ppu_mem.read(0x2000), ppu_mem.read(0x2800));
        
        // 0x2000 and 0x2400 should be the same (vertical mirroring)
        assert_eq!(ppu_mem.read(0x2000), ppu_mem.read(0x2400));
        
        // 0x2800 and 0x2C00 should be the same (vertical mirroring)
        assert_eq!(ppu_mem.read(0x2800), ppu_mem.read(0x2C00));
    }
    
    #[test]
    fn test_palette_mirroring() {
        let mut ppu_mem = PpuMemory::new();
        
        // Write to 0x3F00 and 0x3F10 (mirrors of each other)
        ppu_mem.write(0x3F00, 0x12);
        ppu_mem.write(0x3F10, 0x34);
        
        // 0x3F00 and 0x3F10 should be the same
        assert_eq!(ppu_mem.read(0x3F00), ppu_mem.read(0x3F10));
        
        // Other palette addresses should be independent
        ppu_mem.write(0x3F01, 0x56);
        assert_ne!(ppu_mem.read(0x3F00), ppu_mem.read(0x3F01));
        assert_ne!(ppu_mem.read(0x3F10), ppu_mem.read(0x3F11));
    }
}
