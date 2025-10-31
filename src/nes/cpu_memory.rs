use crate::nes::utils::Memory;
use std::fmt;
use std::ptr;

/// A wrapper around `Nes` that implements `Memory` for the CPU.
/// This is needed to break the circular dependency between `Cpu` and `Nes`.
#[derive(Debug, Clone)]
pub struct CpuMemory {
    nes: *mut dyn Memory,
}

impl CpuMemory {
    /// Create a new `CpuMemory` that wraps the given `Nes`.
    pub fn new(nes: *mut dyn Memory) -> Self {
        CpuMemory { nes }
    }
    
    /// Get a mutable reference to the underlying memory, if available
    fn get_memory(&self) -> Option<&mut dyn Memory> {
        if self.nes.is_null() {
            None
        } else {
            unsafe { Some(&mut *self.nes) }
        }
    }
}

impl Memory for CpuMemory {
    fn read_byte(&self, addr: u16) -> Result<u8, crate::nes::utils::MemoryError> {
        match self.get_memory() {
            Some(memory) => memory.read_byte(addr),
            None => Ok(0), // Default value when no memory is available
        }
    }

    fn write_byte(&mut self, addr: u16, value: u8) -> Result<(), crate::nes::utils::MemoryError> {
        if let Some(memory) = self.get_memory() {
            memory.write_byte(addr, value)?;
        }
        Ok(())
    }
    
    // Implement the remaining required methods from the Memory trait
    fn read_word(&self, addr: u16) -> Result<u16, crate::nes::utils::MemoryError> {
        match self.get_memory() {
            Some(memory) => memory.read_word(addr),
            None => Ok(0), // Default value when no memory is available
        }
    }
    
    fn write_word(&mut self, addr: u16, value: u16) -> Result<(), crate::nes::utils::MemoryError> {
        if let Some(memory) = self.get_memory() {
            memory.write_word(addr, value)?;
        }
        Ok(())
    }
}

// Safety: We ensure that the `Nes` outlives any `CpuMemory` that references it.
unsafe impl Send for CpuMemory {}
unsafe impl Sync for CpuMemory {}
