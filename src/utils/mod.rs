//! Utility functions and types for the NES emulator

// Re-export the bitflags macro from the bitflags crate
pub use bitflags::bitflags;

/// A trait for memory operations
pub trait Memory {
    /// Read a byte from the specified address
    fn read_byte(&self, addr: u16) -> std::io::Result<u8>;

    /// Write a byte to the specified address
    fn write_byte(&mut self, addr: u16, value: u8) -> std::io::Result<()>;

    /// Read a 16-bit word (little-endian) from the specified address
    fn read_word(&self, addr: u16) -> std::io::Result<u16> {
        let lo = self.read_byte(addr)? as u16;
        let hi = self.read_byte(addr.wrapping_add(1))? as u16;
        Ok((hi << 8) | lo)
    }

    /// Write a 16-bit word (little-endian) to the specified address
    fn write_word(&mut self, addr: u16, value: u16) -> std::io::Result<()> {
        self.write_byte(addr, value as u8)?;
        self.write_byte(addr.wrapping_add(1), (value >> 8) as u8)?;
        Ok(())
    }
}

/// A simple implementation of Memory for a byte slice
pub struct MemoryBlock<'a> {
    data: &'a mut [u8],
}

impl<'a> MemoryBlock<'a> {
    /// Create a new MemoryBlock wrapping the given slice
    pub fn new(data: &'a mut [u8]) -> Self {
        MemoryBlock { data }
    }
}

impl Memory for MemoryBlock<'_> {
    fn read_byte(&self, addr: u16) -> std::io::Result<u8> {
        self.data.get(addr as usize).copied().ok_or_else(|| {
            std::io::Error::new(
                std::io::ErrorKind::InvalidInput,
                format!("Address out of bounds: 0x{:04X}", addr),
            )
        })
    }

    fn write_byte(&mut self, addr: u16, value: u8) -> std::io::Result<()> {
        self.data
            .get_mut(addr as usize)
            .map(|v| *v = value)
            .ok_or_else(|| {
                std::io::Error::new(
                    std::io::ErrorKind::InvalidInput,
                    format!("Address out of bounds: 0x{:04X}", addr),
                )
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bitflags() {
        bitflags::bitflags! {
            #[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
            pub struct TestFlags: u8 {
                const FLAG_A = 0b00000001;
                const FLAG_B = 0b00000010;
                const FLAG_C = 0b00000100;
            }
        }

        let mut flags = TestFlags::FLAG_A | TestFlags::FLAG_B;
        assert!(flags.contains(TestFlags::FLAG_A));
        assert!(flags.contains(TestFlags::FLAG_B));
        assert!(!flags.contains(TestFlags::FLAG_C));

        flags.insert(TestFlags::FLAG_C);
        assert!(flags.contains(TestFlags::FLAG_C));

        flags.remove(TestFlags::FLAG_A);
        assert!(!flags.contains(TestFlags::FLAG_A));
    }

    #[test]
    fn test_memory_block() {
        let mut data = [0u8; 4];
        let mut mem = MemoryBlock::new(&mut data);

        // Test writing and reading bytes
        mem.write_byte(0, 0x12).unwrap();
        mem.write_byte(1, 0x34).unwrap();
        assert_eq!(mem.read_byte(0).unwrap(), 0x12);
        assert_eq!(mem.read_byte(1).unwrap(), 0x34);

        // Test writing and reading words
        mem.write_word(2, 0x5678).unwrap();
        assert_eq!(mem.read_word(2).unwrap(), 0x5678);

        // Test out of bounds
        assert!(mem.read_byte(4).is_err());
        assert!(mem.write_byte(4, 0).is_err());
    }
}
