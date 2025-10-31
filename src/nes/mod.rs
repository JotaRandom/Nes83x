//! NES system emulation

pub mod apu;
pub mod cartridge;
pub mod utils;
pub mod cpu;
pub mod cpu_memory;
pub mod input;
pub mod mapper;
pub mod ppu;

use self::cpu::{Cpu, CpuError};
use self::cpu_memory::CpuMemory;
use std::sync::Arc;
use std::sync::Mutex;
use self::ppu::Ppu;
use self::apu::Apu;
use self::mapper::Mapper;
use self::input::{InputManager, Input, Button};

// Re-export the Memory trait
pub use crate::nes::utils::Memory;
use std::io;
use thiserror::Error;
use std::path::Path;

/// Represents the NES system
#[derive(Debug)]
pub struct Nes {
    /// The CPU (Central Processing Unit)
    pub cpu: Cpu<CpuMemory>,
    
    /// The PPU (Picture Processing Unit)
    pub ppu: Ppu,
    
    /// The APU (Audio Processing Unit)
    pub apu: Apu,
    
    /// The cartridge (ROM) and mapper
    pub cart: Option<Box<dyn Mapper + Send + Sync>>,
    
    /// The system RAM (2KB + mirrors)
    pub ram: [u8; 2048],
    
    /// Input manager for both controllers
    pub input: InputManager,
    
    /// Whether the system is running
    pub is_running: bool,
    
    /// Current cycle count (for timing)
    pub cycle_count: u64,
    
    /// CPU memory wrapper (stored separately to avoid circular references)
    cpu_memory: CpuMemory,
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

// Memory trait implementation is now in utils/mod.rs

impl Nes {
    /// Create a new NES system
    pub fn new() -> Self {
        // Create a new NES with uninitialized CPU memory
        let mut nes = Nes {
            cpu: Cpu::new(),
            ppu: Ppu::new(),
            apu: Apu::new(),
            cart: None,
            ram: [0; 2048],
            input: InputManager::new(),
            is_running: false,
            cycle_count: 0,
            cpu_memory: CpuMemory::new(ptr::null_mut()),
        };
        
        // Now that we have a stable address, create CpuMemory with a pointer to self
        let ptr = &mut nes as *mut _ as *mut dyn Memory;
        nes.cpu_memory = CpuMemory::new(ptr);
        
        // Update the CPU's memory reference
        nes.cpu.memory = nes.cpu_memory.clone();
        
        // Reset the system to initialize everything properly
        let _ = nes.reset();
        
        nes
    }
    
    /// Reset the NES to its initial state
    pub fn reset(&mut self) -> Result<(), NesError> {
        self.cpu.reset();
        self.ppu.reset();
        self.apu.reset();
        self.ram = [0; 2048];
        
        // Reset the input manager
        self.input.reset();
        
        self.is_running = false;
        self.cycle_count = 0;
        
        // Reset cartridge if present
        if let Some(cart) = &mut self.cart {
            cart.reset();
        }
        
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
        let _chr_rom = if chr_rom_size > 0 {
            let chr = &rom_data[offset..offset + chr_rom_size];
            let _ = offset; // offset is not used after this point
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
        let _prg_rom = if prg_rom_size == 16 * 1024 {
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
        
        // Run the CPU for one frame (approximately 29780 cycles at 1.79 MHz)
        let target_cycles = 29780;
        let mut cycles = 0;
        
        // Create raw pointers to avoid multiple mutable borrows
        let apu_ptr = &self.apu as *const Apu as *mut Apu;
        let cpu_ptr = &self.cpu as *const super::cpu::Cpu as *mut super::cpu::Cpu;
        
        while cycles < target_cycles {
            // Safe because we know these pointers are valid for the duration of this method
            let cpu = unsafe { &mut *cpu_ptr };
            
            // Step the CPU and handle any NMI that might be triggered
            let cpu_cycles = cpu.step(self)?;
            cycles += cpu_cycles as u32;
            
            // Run the PPU for the same number of cycles * 3 (PPU runs at 3x the CPU speed)
            for _ in 0..(cpu_cycles * 3) {
                let nmi = self.ppu.tick();
                if nmi {
                    cpu.nmi();
                }
            }
            
            // Run the APU for the same number of cycles
            let apu = unsafe { &mut *apu_ptr };
            for _ in 0..cpu_cycles {
                apu.tick(self);
            }
            
            self.cycle_count += cpu_cycles as u64;
        }
        
        // Handle controller input
        self.update_controllers();
        
        Ok(())
    }
    
    /// Set the state of a controller button
    pub fn set_button_state(&mut self, controller: usize, button: Button, pressed: bool) {
        self.input.set_button_state(controller, button, pressed);
    }
    
    /// Read the current state of a controller
    pub fn read_controller(&mut self, port: usize) -> u8 {
        self.input.read(port)
    }
    
    /// Write to the controller port (strobe)
    pub fn write_controller_strobe(&mut self, value: u8) {
        self.input.write(value);
    }
    
    /// Update the state of the controllers
    fn update_controllers(&mut self) {
        // The InputManager updates its state when write() or read() is called.
        // This method is a placeholder for any additional controller updates
        // that might be needed in the future.
    }
}

impl Memory for Nes {
    fn read_byte(&self, addr: u16) -> Result<u8, crate::nes::utils::MemoryError> {
        match addr {
            // RAM (2KB mirrored)
            0x0000..=0x1FFF => Ok(self.ram[(addr & 0x07FF) as usize]),
            
            // PPU registers (mirrored every 8 bytes)
            0x2000..=0x3FFF => {
                use crate::nes::ppu::Ppu;
                let reg = (addr & 0x0007) as u16;
                // Safe because we know ppu is valid for the lifetime of self
                let ppu: &mut Ppu = unsafe { &mut *(&self.ppu as *const Ppu as *mut Ppu) };
                Ok(ppu.read_register(reg))
            }
            
            // APU and I/O registers
            0x4000..=0x4015 | 0x4017 => {
                use crate::nes::apu::Apu;
                // Safe because we know apu is valid for the lifetime of self
                let apu: &Apu = unsafe { &*(&self.apu as *const Apu) };
                // APU read_byte returns a Result<u8, MemoryError>
                apu.read_byte(addr)
            }
            
            // Controller 1
            0x4016 => {
                use crate::nes::input::InputManager;
                // Safe because we know input is valid for the lifetime of self
                let input: &mut InputManager = unsafe { &mut *(&self.input as *const InputManager as *mut InputManager) };
                Ok(input.read(0))
            }
            
            // PRG-ROM (32KB, potentially mirrored)
            0x8000..=0xFFFF => {
                if let Some(ref cart) = self.cart {
                    // read_prg_byte returns a u8 directly
                    Ok(cart.read_prg_byte(addr))
                } else {
                    // If no cartridge is loaded, return a default value (0xFF)
                    Ok(0xFF)
                }
            }
            
            // Unmapped memory
            _ => Ok(0xFF), // Default value for unmapped memory
        }
    }
    
    fn write_byte(&mut self, addr: u16, value: u8) -> Result<(), crate::nes::utils::MemoryError> {
        match addr {
            // RAM (2KB mirrored)
            0x0000..=0x1FFF => {
                self.ram[(addr & 0x07FF) as usize] = value;
                Ok(())
            }
            
            // PPU registers (mirrored every 8 bytes)
            0x2000..=0x3FFF => {
                use crate::nes::ppu::Ppu;
                let reg = (addr & 0x0007) as u16;
                // Safe because we know ppu is valid for the lifetime of self
                let ppu: &mut Ppu = unsafe { &mut *(&self.ppu as *const Ppu as *mut Ppu) };
                ppu.write_register(reg, value);
                Ok(())
            }
            
            // OAM DMA
            0x4014 => {
                // TODO: Implement OAM DMA
                Ok(())
            }
            
            // Controller register 1
            0x4016 => {
                use crate::nes::input::InputManager;
                // Safe because we know input is valid for the lifetime of self
                let input: &mut InputManager = unsafe { &mut *(&self.input as *const InputManager as *mut InputManager) };
                input.write(value);
                Ok(())
            }
            
            // APU and I/O registers
            0x4000..=0x4013 | 0x4015 | 0x4017 => {
                use crate::nes::apu::Apu;
                // Safe because we know apu is valid for the lifetime of self
                let apu: &mut Apu = unsafe { &mut *(&self.apu as *const Apu as *mut Apu) };
                apu.write_register(addr, value);
                Ok(())
            }
            
            // PRG-ROM (read-only)
            0x8000..=0xFFFF => {
                // ROM is read-only, but some mappers might handle writes
                // For now, just ignore writes to ROM
                Ok(())
            }
            
            // Unmapped memory - ignore writes
            _ => Ok(()),
        }
    }
    
}

#[cfg(test)]
mod tests {
    use super::*;

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
