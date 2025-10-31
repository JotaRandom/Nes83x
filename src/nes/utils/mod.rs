use std::fmt;
use std::io;

/// A trait for memory operations
pub trait Memory: fmt::Debug {
    /// Read a byte from memory
    fn read_byte(&self, addr: u16) -> io::Result<u8>;
    
    /// Write a byte to memory
    fn write_byte(&mut self, addr: u16, value: u8) -> io::Result<()>;
    
    /// Read a word (little-endian) from memory
    fn read_word(&self, addr: u16) -> io::Result<u16> {
        let lo = self.read_byte(addr)? as u16;
        let hi = self.read_byte(addr.wrapping_add(1))? as u16;
        Ok((hi << 8) | lo)
    }
    
    /// Write a word (little-endian) to memory
    fn write_word(&mut self, addr: u16, value: u16) -> io::Result<()> {
        self.write_byte(addr, value as u8)?;
        self.write_byte(addr.wrapping_add(1), (value >> 8) as u8)
    }
}

// Implement Debug for dyn Memory
impl fmt::Debug for dyn Memory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Memory {{ ... }}")
    }
}
