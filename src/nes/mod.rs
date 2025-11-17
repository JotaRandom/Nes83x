//! NES system emulation

pub mod apu;
pub mod cartridge;
pub mod cpu;
pub mod input;
pub mod mapper;
pub mod ppu;
pub mod utils;

use self::apu::Apu;
use self::cpu::{Cpu, CpuError};
use self::mapper::Mapper;
use self::ppu::Ppu;

// Re-export the Memory trait
pub use crate::nes::utils::Memory;
use log::{info, error};
use std::cell::RefCell;
use std::path::Path;
use std::rc::Rc;
use thiserror::Error;

/// Dummy memory implementation for initialization
#[derive(Debug)]
struct DummyMemory;

impl Memory for DummyMemory {
    fn read_byte(&self, _addr: u16) -> Result<u8, crate::nes::utils::MemoryError> {
        Ok(0)
    }
    fn write_byte(&mut self, _addr: u16, _value: u8) -> Result<(), crate::nes::utils::MemoryError> {
        Ok(())
    }
}

/// Represents the NES system
#[derive(Debug)]
pub struct Nes {
    /// The CPU (Central Processing Unit)
    pub cpu: Cpu,

    /// The PPU (Picture Processing Unit)
    pub ppu: Ppu,

    /// The APU (Audio Processing Unit)
    pub apu: Apu,

    /// The cartridge (ROM) and mapper
    pub cart: Option<Rc<RefCell<dyn Mapper>>>,

    /// Input manager for controllers
    pub input: input::InputManager,

    /// The system RAM (2KB + mirrors)
    pub ram: [u8; 2048],

    /// Whether the system is running
    pub is_running: bool,

    /// Current cycle count (for timing)
    pub cycle_count: u64,

    // CPU memory wrapper (removed - CPU now accesses NES directly)
    // cpu_memory: Option<CpuMemory>,
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
            input: input::InputManager::new(),
            ram: [0; 2048],
            is_running: false,
            cycle_count: 0,
            // cpu_memory: None,
        };

        // Now that we have a stable address, create CpuMemory with a pointer to self
        // nes.cpu_memory = Some(CpuMemory::new(&mut nes as *mut _ as *mut dyn Memory));

        // Update the CPU's memory reference
        // nes.cpu.memory = nes.cpu_memory.clone();

        // Reset the system to initialize everything properly
        let _ = nes.reset();

        nes
    }

    /// Reset the NES to its initial state
    pub fn reset(&mut self) -> Result<(), NesError> {
        self.cpu.reset();
        self.ppu.reset();
        self.apu.reset();
        self.input.reset();
        self.ram = [0; 2048];

        self.is_running = false;
        self.cycle_count = 0;

        // Reset cartridge if present
        if let Some(cart) = &mut self.cart {
            cart.borrow_mut().reset();
        }

        Ok(())
    }

    /// Load a ROM from raw bytes
    pub fn load_rom_from_bytes(&mut self, data: &[u8]) -> Result<(), NesError> {
        info!("ROM file size: {} bytes", data.len());

        // Basic iNES header check
        if data.len() < 16 {
            error!("ROM file too small: {} bytes", data.len());
            return Err(NesError::InvalidRom);
        }

        let header = &data[0..4];
        info!("ROM header: {:?}", header);

        if header != b"NES\x1A" {
            error!("Invalid NES header: expected NES\\x1A, got {:?}", header);
            return Err(NesError::InvalidRom);
        }

        info!("Valid NES header detected");

        let prg_rom_size = data[4] as usize * 16 * 1024; // 16KB units
        let chr_rom_size = data[5] as usize * 8 * 1024; // 8KB units

        info!("PRG ROM size: {} bytes, CHR ROM size: {} bytes", prg_rom_size, chr_rom_size);

        let flags6 = data[6];
        let mapper_lower = (flags6 & 0xF0) >> 4;
        let mirroring = if (flags6 & 0x01) != 0 {
            mapper::Mirroring::Vertical
        } else {
            mapper::Mirroring::Horizontal
        };
        let _has_battery = (flags6 & 0x02) != 0;
        let _has_trainer = (flags6 & 0x04) != 0;
        let _four_screen = (flags6 & 0x08) != 0;

        let flags7 = data[7];
        let mapper_upper = flags7 & 0xF0;
        let mapper = mapper_upper | mapper_lower;

        info!("Mapper: {}, Mirroring: {:?}", mapper, mirroring);

        // Skip header (16 bytes) and trainer (if present)
        let mut offset = 16;
        if _has_trainer {
            offset += 512;
        }

        // Load PRG ROM
        let prg_rom = data[offset..offset + prg_rom_size].to_vec();
        offset += prg_rom_size;

        // Load CHR ROM (if present, otherwise we'll use CHR RAM)
        let chr_rom = if chr_rom_size > 0 {
            Some(data[offset..offset + chr_rom_size].to_vec())
        } else {
            None
        };

        // Create the mapper
        let mapper = mapper::create_mapper(mapper, prg_rom, chr_rom, mirroring)?;

        // Set the cartridge
        self.cart = Some(mapper);

        // Configure the mapper in PPU memory
        if let Some(ref cart) = self.cart {
            self.ppu.memory.set_mapper(Rc::clone(cart));
            let ppu_mirroring = match mirroring {
                mapper::Mirroring::Horizontal => mapper::Mirroring::Horizontal,
                mapper::Mirroring::Vertical => mapper::Mirroring::Vertical,
                mapper::Mirroring::FourScreen => mapper::Mirroring::FourScreen,
                mapper::Mirroring::SingleScreen(n) => mapper::Mirroring::SingleScreen(n),
            };
            self.ppu.memory.set_mirroring(ppu_mirroring);
        }

        // Reset the system to initialize everything properly
        self.reset()?;

        info!("ROM loaded successfully");

        Ok(())
    }

    /// Load a ROM file
    pub fn load_rom<P: AsRef<Path>>(&mut self, path: P) -> Result<(), NesError> {
        let data = std::fs::read(path)?;
        self.load_rom_from_bytes(&data)
    }

    /// Set the state of a button on a controller
    pub fn set_button_state(&mut self, controller: usize, button: input::Button, pressed: bool) {
        self.input.set_button_state(controller, button, pressed);
    }

    /// Run the emulation for a single frame
    pub fn run_frame(&mut self) -> Result<(), NesError> {
        if !self.is_running {
            return Ok(());
        }

        let frame = self.ppu.frame();
        if frame % 60 == 0 {
            eprintln!("Running frame {}", frame);
        }

        // Run the CPU for one frame (approximately 29780 cycles at 1.79 MHz)
        let target_cycles = 29780;
        let mut cycles = 0;

        while cycles < target_cycles {
            // Step the CPU
            let cpu_cycles = self.cpu.step(self)?;
            cycles += cpu_cycles as u32;

            // Run the PPU for the same number of cycles * 3 (PPU runs at 3x the CPU speed)
            for _ in 0..(cpu_cycles * 3) {
                let nmi = self.ppu.tick();
                if nmi {
                    self.cpu.nmi();
                }
            }

            // Run the APU for the same number of cycles
            for _ in 0..cpu_cycles {
                self.apu.tick(self);
            }

            self.cycle_count += cpu_cycles as u64;
        }

        Ok(())
    }
}

impl Memory for Nes {
    fn read_byte(&self, addr: u16) -> Result<u8, crate::nes::utils::MemoryError> {
        match addr {
            // RAM (2KB mirrored)
            0x0000..=0x1FFF => Ok(self.ram[(addr & 0x07FF) as usize]),

            // PPU registers (mirrored every 8 bytes)
            0x2000..=0x3FFF => {
                let reg = (addr & 0x0007) as u16;
                Ok(self.ppu.read_register(reg))
            }

            // APU and I/O registers
            0x4000..=0x4015 => self.apu.read_byte(addr),

            // Controller 1 (0x4016) and Controller 2 (0x4017)
            0x4016 => Ok(self.input.read(0)),
            0x4017 => Ok(self.input.read(1)),

            // PRG-ROM (32KB, potentially mirrored)
            0x8000..=0xFFFF => {
                if let Some(ref cart) = self.cart {
                    // read_prg_byte returns a u8 directly
                    Ok(cart.borrow().read(addr))
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
                let reg = (addr & 0x0007) as u16;
                self.ppu.write_register(reg, value);
                Ok(())
            }

            // OAM DMA
            0x4014 => {
                // TODO: Implement OAM DMA
                Ok(())
            }

            // Controller register 1 (strobe)
            0x4016 => {
                self.input.write(value);
                Ok(())
            }

            // APU and I/O registers
            0x4000..=0x4013 | 0x4015 | 0x4017 => {
                self.apu.write_register(addr, value);
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
