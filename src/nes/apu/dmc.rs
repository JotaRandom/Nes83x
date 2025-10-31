//! NES DMC (Delta Modulation Channel) implementation

use super::DMC_RATE_TABLE;
use crate::nes::Memory;

/// DMC channel (Delta Modulation Channel)
#[derive(Debug, Default)]
pub struct Dmc {
    // Registers
    pub irq_enabled: bool,
    pub loop_flag: bool,
    pub rate_index: u8,          // 4 bits (0-15)
    pub direct_load: u8,         // 7 bits (0-127)
    pub sample_address: u16,     // 11 bits (0x4000-0xFFFF)
    pub sample_length: u16,      // 12 bits (1-4096)
    
    // Internal state
    pub enabled: bool,
    pub current_address: u16,
    pub bytes_remaining: u16,
    sample_buffer: Option<u8>,
    shift_register: u8,
    bit_counter: u8,         // 3 bits (0-7)
    silence: bool,
    output_level: u8,        // 7 bits (0-127)
    tick_period: u16,
    tick_value: u16,
    irq_pending: bool,
}

impl Dmc {
    /// Create a new DMC channel
    pub fn new() -> Self {
        let mut dmc = Dmc {
            irq_enabled: false,
            loop_flag: false,
            rate_index: 0,
            direct_load: 0,
            sample_address: 0,
            sample_length: 0,
            enabled: false,
            current_address: 0,
            bytes_remaining: 0,
            sample_buffer: None,
            shift_register: 0,
            bit_counter: 0,
            silence: true,
            output_level: 0,
            tick_period: 0,
            tick_value: 0,
            irq_pending: false,
        };
        dmc.reset();
        dmc
    }
    
    /// Reset the DMC channel to its initial state
    pub fn reset(&mut self) {
        self.irq_enabled = false;
        self.loop_flag = false;
        self.rate_index = 0;
        self.direct_load = 0;
        self.sample_address = 0;
        self.sample_length = 0;
        self.enabled = false;
        self.current_address = 0;
        self.bytes_remaining = 0;
        self.sample_buffer = None;
        self.shift_register = 0;
        self.bit_counter = 0;
        self.silence = true;
        self.output_level = 0;
        self.tick_period = 0;
        self.tick_value = 0;
        self.irq_pending = false;
    }
    
    /// Fetch a byte from memory at the current address
    fn fetch_byte(&mut self, memory: &mut dyn Memory) -> Result<Option<u8>, crate::nes::utils::MemoryError> {
        if self.bytes_remaining == 0 {
            return Ok(None);
        }
        
        // Read the byte from memory
        let byte = memory.read_byte(0x8000 | self.current_address)?;
        
        // Update the address and remaining bytes
        self.current_address = if self.current_address == 0xFFFF {
            0x8000
        } else {
            self.current_address.wrapping_add(1)
        };
        
        self.bytes_remaining = self.bytes_remaining.wrapping_sub(1);
        
        // Handle loop if enabled and we've reached the end
        if self.bytes_remaining == 0 && self.loop_flag {
            self.current_address = self.sample_address;
            self.bytes_remaining = self.sample_length;
        }
        
        // Trigger IRQ if enabled and we've reached the end
        if self.bytes_remaining == 0 && self.irq_enabled {
            self.irq_pending = true;
        }
        
        Ok(Some(byte))
    }
    
    /// Write to control register ($4010)
    pub fn write_control(&mut self, value: u8) {
        self.irq_enabled = (value & 0x80) != 0;
        self.loop_flag = (value & 0x40) != 0;
        self.rate_index = value & 0x0F;
        self.tick_period = DMC_RATE_TABLE[self.rate_index as usize];
        
        // Clear IRQ flag if IRQ is disabled
        if !self.irq_enabled {
            self.irq_pending = false;
        }
    }
    
    /// Write to direct load register ($4011)
    pub fn write_value(&mut self, value: u8) {
        self.direct_load = value & 0x7F;
    }
    
    /// Write to sample address register ($4012)
    pub fn write_address(&mut self, value: u8) {
        // Sample address = 0xC000 + (value * 64)
        self.sample_address = 0xC000 | ((value as u16) << 6);
    }
    
    /// Write to sample length register ($4013)
    pub fn write_length(&mut self, value: u8) {
        // Sample length = (value * 16) + 1
        self.sample_length = ((value as u16) << 4) | 0x0001;
    }
    
    /// Clock the DMC channel
    pub fn clock(&mut self, memory: &mut dyn Memory) -> Result<u8, crate::nes::utils::MemoryError> {
        let mut irq_occurred = 0;
        
        // Only process if enabled
        if !self.enabled {
            return Ok(0);
        }
        
        // Clock the timer
        if self.tick_value > 0 {
            self.tick_value -= 1;
        } else {
            // Reset the timer
            self.tick_value = self.tick_period;
            
            // Clock the output unit
            self.clock_output_unit();
        }
        
        // Check if we need to start a new sample
        if self.sample_buffer.is_none() && self.bytes_remaining > 0 {
            // Start memory read
            self.fetch_sample(memory)?;
        }
        
        // Check for IRQ
        if self.irq_pending && self.irq_enabled {
            irq_occurred = 0x80;
        }
        
        Ok(irq_occurred)
    }
    
    /// Clock the output unit
    fn clock_output_unit(&mut self) {
        // Only process if not silenced
        if !self.silence {
            let delta = (self.shift_register & 0x01) as u8;
            
            if delta != 0 {
                if self.output_level <= 125 {
                    self.output_level += 2;
                }
            } else {
                if self.output_level >= 2 {
                    self.output_level -= 2;
                }
            }
            
            // Shift the register right
            self.shift_register >>= 1;
        }
        
        // Decrement bit counter
        if self.bit_counter > 0 {
            self.bit_counter -= 1;
        } else {
            // Start a new output cycle
            self.bit_counter = 7;
            
            if let Some(sample) = self.sample_buffer.take() {
                self.shift_register = sample;
                self.silence = false;
            } else {
                self.silence = true;
            }
        }
    }
    
    /// Fetch a sample from memory
    pub fn fetch_sample(&mut self, memory: &mut dyn Memory) -> Result<(), crate::nes::utils::MemoryError> {
        if let Some(byte) = self.fetch_byte(memory)? {
            self.sample_buffer = Some(byte);
            self.silence = false;
            self.bit_counter = 8;
            self.shift_register = byte;
        } else {
            self.silence = true;
        }
        
        Ok(())
    }
    
    /// Restart the DMC channel
    pub fn restart(&mut self) {
        self.current_address = self.sample_address;
        self.bytes_remaining = self.sample_length;
    }
    
    /// Enable or disable the DMC channel
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
        
        if !enabled {
            self.bytes_remaining = 0;
        }
    }
    
    /// Get the current output level (0-127)
    pub fn output(&self) -> u8 {
        self.output_level
    }
    
    /// Check if an IRQ is pending
    pub fn is_irq_pending(&self) -> bool {
        self.irq_pending
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;
    use std::rc::Rc;

    struct TestMemory {
        data: Rc<RefCell<[u8; 0x10000]>>,
    }
    
    impl TestMemory {
        fn new() -> Self {
            TestMemory {
                data: Rc::new(RefCell::new([0; 0x10000])),
            }
        }
        
        fn write(&self, addr: u16, value: u8) {
            self.data.borrow_mut()[addr as usize] = value;
        }
    }
    
    impl Memory for TestMemory {
        fn read_byte(&self, addr: u16) -> std::io::Result<u8> {
            Ok(self.data.borrow()[addr as usize])
        }
        
        fn write_byte(&mut self, _addr: u16, _value: u8) -> std::io::Result<()> {
            Ok(())
        }
    }
    
    #[test]
    fn test_dmc_initialization() {
        let dmc = Dmc::new();
        assert!(!dmc.enabled);
        assert_eq!(dmc.output_level, 0);
    }
    
    #[test]
    fn test_dmc_control_register() {
        let mut dmc = Dmc::new();
        
        // Test control settings
        dmc.write_control(0xFF);
        assert!(dmc.irq_enabled);
        assert!(dmc.loop_flag);
        assert_eq!(dmc.rate_index, 0x0F);
        
        // Test IRQ disable
        dmc.irq_pending = true;
        dmc.write_control(0x7F);
        assert!(!dmc.irq_enabled);
        assert!(!dmc.irq_pending); // Should clear IRQ
    }
    
    #[test]
    fn test_dmc_sample_address() {
        let mut dmc = Dmc::new();
        
        // Test sample address calculation
        dmc.write_address(0x80);
        assert_eq!(dmc.sample_address, 0xC000 | (0x80 << 6));
    }
    
    #[test]
    fn test_dmc_sample_length() {
        let mut dmc = Dmc::new();
        
        // Test sample length calculation
        dmc.write_length(0x01);
        assert_eq!(dmc.sample_length, (1 << 4) | 1);
    }
    
    #[test]
    fn test_dmc_clock() {
        let mut dmc = Dmc::new();
        let memory = TestMemory::new();
        
        // Set up DMC
        dmc.write_control(0x0F); // Fastest rate
        dmc.write_address(0x00);
        dmc.write_length(0x01);
        dmc.set_enabled(true);
        
        // Write a sample to memory
        memory.write(0xC000, 0xAA);
        
        // Clock until sample is loaded
        for _ in 0..100 {
            dmc.clock(&memory);
        }
        
        // Output level should have changed
        assert_ne!(dmc.output_level, 0);
    }
}
