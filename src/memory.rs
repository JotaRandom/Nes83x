//! Standardized memory interface for the NES emulator

use std::fmt;
use thiserror::Error;

/// Errors that can occur during memory operations
#[derive(Error, Debug)]
pub enum MemoryError {
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),
    
    #[error("Invalid memory access at address: {0:04X}")]
    InvalidAccess(u16),
    
    #[error("Memory operation not supported")]
    NotSupported,
}

/// A trait for memory operations
pub trait Memory: fmt::Debug + Send + Sync {
    /// Read a byte from memory
    fn read_byte(&self, addr: u16) -> Result<u8, MemoryError>;
    
    /// Write a byte to memory
    fn write_byte(&mut self, addr: u16, value: u8) -> Result<(), MemoryError>;
    
    /// Read a word (little-endian) from memory
    fn read_word(&self, addr: u16) -> Result<u16, MemoryError> {
        let lo = self.read_byte(addr)? as u16;
        let hi = self.read_byte(addr.wrapping_add(1))? as u16;
        Ok((hi << 8) | lo)
    }
    
    /// Write a word (little-endian) to memory
    fn write_word(&mut self, addr: u16, value: u16) -> Result<(), MemoryError> {
        self.write_byte(addr, value as u8)?;
        self.write_byte(addr.wrapping_add(1), (value >> 8) as u8)
    }
}

/// A simple implementation of Memory for a byte slice
#[derive(Debug)]
pub struct MemoryBlock {
    data: Vec<u8>,
}

impl MemoryBlock {
    /// Create a new MemoryBlock with the given size
    pub fn new(size: usize) -> Self {
        Self {
            data: vec![0; size],
        }
    }
    
    /// Create a new MemoryBlock from an existing slice
    pub fn from_slice(data: &[u8]) -> Self {
        Self {
            data: data.to_vec(),
        }
    }
    
    /// Get a reference to the underlying data
    pub fn as_slice(&self) -> &[u8] {
        &self.data
    }
    
    /// Get a mutable reference to the underlying data
    pub fn as_mut_slice(&mut self) -> &mut [u8] {
        &mut self.data
    }
}

impl Memory for MemoryBlock {
    fn read_byte(&self, addr: u16) -> Result<u8, MemoryError> {
        self.data
            .get(addr as usize)
            .copied()
            .ok_or_else(|| MemoryError::InvalidAccess(addr))
    }
    
    fn write_byte(&mut self, addr: u16, value: u8) -> Result<(), MemoryError> {
        if let Some(byte) = self.data.get_mut(addr as usize) {
            *byte = value;
            Ok(())
        } else {
            Err(MemoryError::InvalidAccess(addr))
        }
    }
}

// Implement Memory for Box<dyn Memory> to allow for dynamic dispatch
impl Memory for Box<dyn Memory> {
    fn read_byte(&self, addr: u16) -> Result<u8, MemoryError> {
        self.as_ref().read_byte(addr)
    }
    
    fn write_byte(&mut self, addr: u16, value: u8) -> Result<(), MemoryError> {
        self.as_mut().write_byte(addr, value)
    }
}

// Implement Debug for dyn Memory
impl fmt::Debug for dyn Memory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Memory {{ ... }}")
    }
}
