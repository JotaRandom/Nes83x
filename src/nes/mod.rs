//! NES system emulation

use crate::cpu::{Cpu, Memory, CpuError};
use thiserror::Error;
use std::path::Path;

/// Represents the NES system
pub struct Nes {
    /// The CPU
    pub cpu: Cpu,
    
    /// The PPU (Picture Processing Unit)
    // ppu: Ppu,
    
    /// The APU (Audio Processing Unit)
    // apu: Apu,
    
    /// The cartridge (ROM)
    // cart: Cartridge,
    
    /// The system RAM (2KB + mirrors)
    ram: [u8; 2048],
    
    /// Controller states
    controllers: [u8; 2],
    
    /// Whether the system is running
    is_running: bool,
}

/// Errors that can occur during NES operations
#[derive(Error, Debug)]
pub enum NesError {
    #[error("CPU error: {0}")]
    CpuError(#[from] CpuError),
    
    #[error("I/O error: {0}")]
    IoError(#[from] std::io::Error),
    
    #[error("Invalid ROM file")]
    InvalidRom,
    
    #[error("Unsupported mapper: {0}")]
    UnsupportedMapper(u8),
}

impl Nes {
    /// Create a new NES system
    pub fn new() -> Self {
        Nes {
            cpu: Cpu::new(),
            // ppu: Ppu::new(),
            // apu: Apu::new(),
            // cart: Cartridge::empty(),
            ram: [0; 2048],
            controllers: [0; 2],
            is_running: false,
        }
    }
    
    /// Reset the NES to its initial state
    pub fn reset(&mut self) -> Result<(), NesError> {
        // Reset CPU
        self.cpu.reset();
        
        // Reset PPU and APU
        // self.ppu.reset();
        // self.apu.reset();
        
        // Clear RAM
        self.ram = [0; 2048];
        
        // Reset controllers
        self.controllers = [0; 2];
        
        // Set PC to reset vector
        // let reset_vector = self.read_word(0xFFFC)?;
        // self.cpu.reg.pc = reset_vector;
        
        self.is_running = true;
        
        Ok(())
    }
    
    /// Load a ROM file
    pub fn load_rom<P: AsRef<Path>>(&mut self, path: P) -> Result<(), NesError> {
        // Load ROM file
        let rom_data = std::fs::read(path)?;
        
        // Basic iNES header check
        if rom_data.len() < 16 || &rom_data[0..4] != b"NES\x1A" {
            return Err(NesError::InvalidRom);
        }
        
        let prg_rom_size = rom_data[4] as usize * 16 * 1024; // 16KB units
        let chr_rom_size = rom_data[5] as usize * 8 * 1024;   // 8KB units
        
        let flags6 = rom_data[6];
        let mapper_lower = (flags6 & 0xF0) >> 4;
        let _mirroring = (flags6 & 0x01) != 0;
        let _has_battery = (flags6 & 0x02) != 0;
        let _has_trainer = (flags6 & 0x04) != 0;
        let _four_screen = (flags6 & 0x08) != 0;
        
        let flags7 = rom_data[7];
        let mapper_upper = flags7 & 0xF0;
        let mapper = mapper_upper | mapper_lower;
        
        // For now, we only support NROM (mapper 0)
        if mapper != 0 {
            return Err(NesError::UnsupportedMapper(mapper));
        }
        
        // Skip header (16 bytes) and trainer (if present)
        let mut offset = 16;
        if _has_trainer {
            offset += 512;
        }
        
        // Load PRG ROM
        let prg_rom = &rom_data[offset..offset + prg_rom_size];
        offset += prg_rom_size;
        
        // Load CHR ROM (if present, otherwise we'll use CHR RAM)
        let chr_rom = if chr_rom_size > 0 {
            let chr = &rom_data[offset..offset + chr_rom_size];
            offset += chr_rom_size;
            Some(chr.to_vec())
        } else {
            None
        };
        
        // For NROM, we need to handle the mirroring of PRG ROM
        // If only 16KB of PRG ROM, it's mirrored to $8000-$BFFF and $C000-$FFFF
        // If 32KB, it's mapped to $8000-$FFFF without mirroring
        
        // TODO: Set up memory mapping for the cartridge
        // For now, we'll just copy the PRG ROM to the appropriate memory locations
        
        // Copy PRG ROM to CPU memory
        // If PRG ROM is 16KB, mirror it to both banks
        let prg_rom = if prg_rom_size == 16 * 1024 {
            let mut mirrored = Vec::with_capacity(32 * 1024);
            mirrored.extend_from_slice(prg_rom);
            mirrored.extend_from_slice(prg_rom);
            mirrored
        } else {
            prg_rom.to_vec()
        };
        
        // TODO: Copy PRG ROM to CPU memory
        
        // Set reset vector
        // let reset_vector = u16::from_le_bytes([prg_rom[0xFFFC - 0x8000], prg_rom[0xFFFD - 0x8000]]);
        // self.cpu.reg.pc = reset_vector;
        
        // TODO: Set up PPU with CHR ROM/RAM
        
        Ok(())
    }
    
    /// Run the emulation for a single frame
    pub fn run_frame(&mut self) -> Result<(), NesError> {
        if !self.is_running {
            return Ok(());
        }
        
        // Run CPU until we've simulated a full frame
        // On NTSC, a frame is 29780.5 CPU cycles (approximately)
        let target_cycles = 29780.5;
        let mut cycles = 0.0;
        
        while cycles < target_cycles {
            // Execute a single CPU instruction
            let cpu_cycles = self.cpu.step(self)?;
            cycles += cpu_cycles as f64;
            
            // TODO: Run PPU for the same number of cycles
            // self.ppu.step(cpu_cycles * 3); // PPU runs at 3x CPU speed
            
            // TODO: Run APU
            // self.apu.step(cpu_cycles);
            
            // TODO: Handle NMI/IRQ
        }
        
        // TODO: Update screen, handle input, etc.
        
        Ok(())
    }
    
    /// Set the state of a controller button
    pub fn set_button_state(&mut self, controller: usize, button: usize, pressed: bool) {
        if controller < 2 && button < 8 {
            if pressed {
                self.controllers[controller] |= 1 << button;
            } else {
                self.controllers[controller] &= !(1 << button);
            }
        }
    }
    
    /// Read the current state of the controllers
    pub fn read_controller(&mut self, port: usize) -> u8 {
        if port < 2 {
            // For simplicity, we'll just return the current state
            // In a real emulator, you'd implement proper controller strobe behavior
            self.controllers[port]
        } else {
            0
        }
    }
}

impl Memory for Nes {
    fn read_byte(&self, addr: u16) -> std::io::Result<u8> {
        match addr {
            // RAM (2KB mirrored)
            0x0000..=0x1FFF => Ok(self.ram[(addr & 0x07FF) as usize]),
            
            // PPU registers
            0x2000..=0x3FFF => {
                let reg = (addr & 0x0007) as u8;
                // self.ppu.read_register(reg)
                Ok(0) // Placeholder
            }
            
            // APU and I/O registers
            0x4000..=0x4017 => {
                // self.apu.read_register(addr)
                Ok(0) // Placeholder
            }
            
            // APU and I/O functionality that is normally disabled
            0x4018..=0x401F => Ok(0),
            
            // Cartridge space (PRG ROM, PRG RAM, and mapper registers)
            0x4020..=0xFFFF => {
                // self.cart.read_prg(addr)
                Ok(0) // Placeholder
            }
        }
    }
    
    fn write_byte(&mut self, addr: u16, value: u8) -> std::io::Result<()> {
        match addr {
            // RAM (2KB mirrored)
            0x0000..=0x1FFF => {
                self.ram[(addr & 0x07FF) as usize] = value;
                Ok(())
            }
            
            // PPU registers
            0x2000..=0x3FFF => {
                let reg = (addr & 0x0007) as u8;
                // self.ppu.write_register(reg, value)
                Ok(()) // Placeholder
            }
            
            // APU and I/O registers
            0x4000..=0x4013 | 0x4015 | 0x4017 => {
                // self.apu.write_register(addr, value)
                Ok(()) // Placeholder
            }
            
            // Controller registers
            0x4016 | 0x4017 => {
                // Handle controller strobe
                // if value & 0x01 != 0 {
                //     self.controller_strobe = true;
                //     self.controller_shift[0] = self.controllers[0];
                //     self.controller_shift[1] = self.controllers[1];
                // } else {
                //     self.controller_strobe = false;
                // }
                Ok(())
            }
            
            // APU and I/O functionality that is normally disabled
            0x4014 => {
                // OAM DMA
                // self.ppu.oam_dma(self, value);
                Ok(())
            }
            
            0x4018..=0x401F => Ok(()), // Unused
            
            // Cartridge space (PRG ROM, PRG RAM, and mapper registers)
            0x4020..=0xFFFF => {
                // self.cart.write_prg(addr, value)
                Ok(()) // Placeholder
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;
    
    #[test]
    fn test_nes_reset() {
        let mut nes = Nes::new();
        nes.reset().unwrap();
        
        assert!(nes.is_running);
        // Add more assertions as needed
    }
    
    // Note: For testing ROM loading, you would need a test ROM file
    // or create a mock ROM in memory
}
