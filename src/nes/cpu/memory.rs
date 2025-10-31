//! CPU Memory implementation for the 6502 CPU

use crate::memory::{self, Memory, MemoryError};
use std::fmt;

/// A wrapper that implements Memory for the CPU
/// This is used to provide a consistent interface for memory access
#[derive(Debug)]
pub struct CpuMemory {
    memory: Box<dyn Memory>,
}

impl CpuMemory {
    /// Create a new CpuMemory that wraps the given Memory
    pub fn new(memory: Box<dyn Memory>) -> Self {
        Self { memory }
    }
    
    /// Get a mutable reference to the underlying memory
    pub fn memory_mut(&mut self) -> &mut dyn Memory {
        &mut *self.memory
    }
}

impl Memory for CpuMemory {
    fn read_byte(&self, addr: u16) -> Result<u8, MemoryError> {
        self.memory.read_byte(addr)
    }

    fn write_byte(&mut self, addr: u16, value: u8) -> Result<(), MemoryError> {
        self.memory.write_byte(addr, value)
    }
    
    fn read_word(&self, addr: u16) -> Result<u16, MemoryError> {
        self.memory.read_word(addr)
    }
    
    fn write_word(&mut self, addr: u16, value: u16) -> Result<(), MemoryError> {
        self.memory.write_word(addr, value)
    }
}

impl fmt::Display for CpuMemory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "CpuMemory")
    }
}
