use std::fmt;

/// A trait for memory operations
pub trait Memory: fmt::Debug {
    /// Read a byte from the specified address
    fn read_byte(&self, addr: u16) -> std::io::Result<u8>;
    
    /// Write a byte to the specified address
    fn write_byte(&mut self, addr: u16, value: u8) -> std::io::Result<()>;
}

// Implement Debug for dyn Memory
impl fmt::Debug for dyn Memory {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Memory {{ ... }}")
    }
}
