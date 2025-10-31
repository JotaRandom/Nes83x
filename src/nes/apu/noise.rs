//! NES Noise Channel implementation

use super::{LENGTH_TABLE, NOISE_PERIOD_TABLE};

/// Noise channel (pseudo-random noise generator)
#[derive(Debug, Default)]
pub struct Noise {
    // Registers
    pub halt_length_counter: bool,
    pub constant_volume: bool,
    pub volume: u8,              // 4 bits (0-15)
    pub mode_flag: bool,
    pub period_index: u8,        // 4 bits (0-15)
    pub length_counter: u8,
    
    // Internal state
    pub enabled: bool,
    shift_register: u16,     // 15-bit LFSR
    timer: u16,
    envelope_start: bool,
    envelope_divider: u8,
    envelope_decay: u8,
    envelope_volume: u8,
}

impl Noise {
    /// Create a new Noise channel
    pub fn new() -> Self {
        // Initialize shift register to 1 (all bits 0 is invalid)
        Noise {
            halt_length_counter: false,
            constant_volume: false,
            volume: 0,
            mode_flag: false,
            period_index: 0,
            length_counter: 0,
            enabled: false,
            shift_register: 1, // Must be non-zero
            timer: 0,
            envelope_start: false,
            envelope_divider: 0,
            envelope_decay: 0,
            envelope_volume: 0,
        }
    }
    
    /// Write to control register ($400C)
    pub fn write_control(&mut self, value: u8) {
        self.halt_length_counter = (value & 0x20) != 0;
        self.constant_volume = (value & 0x10) != 0;
        self.volume = value & 0x0F;
        
        if self.halt_length_counter {
            self.length_counter = 0;
        }
    }
    
    /// Write to period register ($400E)
    pub fn write_period(&mut self, value: u8) {
        self.mode_flag = (value & 0x80) != 0;
        self.period_index = value & 0x0F;
    }
    
    /// Write to length counter register ($400F)
    pub fn write_length_counter(&mut self, value: u8) {
        // Reload length counter if enabled
        if self.enabled {
            let index = (value >> 3) & 0x1F;
            self.length_counter = LENGTH_TABLE[index as usize];
        }
        
        // Restart envelope
        self.envelope_start = true;
    }
    
    /// Clock the timer
    pub fn clock_timer(&mut self) {
        if self.timer == 0 {
            self.timer = NOISE_PERIOD_TABLE[self.period_index as usize];
            self.clock_shift_register();
        } else {
            self.timer -= 1;
        }
    }
    
    /// Clock the shift register
    fn clock_shift_register(&mut self) {
        let feedback = if self.mode_flag {
            // 6-bit mode (bit 0 XOR bit 6)
            (self.shift_register & 0x01) ^ ((self.shift_register >> 6) & 0x01)
        } else {
            // 1-bit mode (bit 0 XOR bit 1)
            (self.shift_register & 0x01) ^ ((self.shift_register >> 1) & 0x01)
        };
        
        // Shift right and set bit 14 to feedback
        self.shift_register >>= 1;
        self.shift_register |= feedback << 14;
        
        // Ensure the shift register never becomes zero
        if self.shift_register == 0 {
            self.shift_register = 1;
        }
    }
    
    /// Clock the envelope generator
    pub fn clock_envelope(&mut self) {
        if self.envelope_start {
            // Start a new envelope
            self.envelope_volume = 15;
            self.envelope_divider = self.volume;
            self.envelope_decay = 15;
            self.envelope_start = false;
        } else if self.envelope_divider > 0 {
            // Decrement the divider
            self.envelope_divider -= 1;
        } else {
            // Reset the divider
            self.envelope_divider = self.volume;
            
            if self.envelope_decay > 0 {
                // Decrement the decay level
                self.envelope_decay -= 1;
            } else if self.halt_length_counter {
                // If the length counter is halted, the decay level is set to 15
                self.envelope_decay = 15;
            }
            
            // Update the envelope volume
            self.envelope_volume = self.envelope_decay;
        }
    }
    
    /// Clock the length counter
    pub fn clock_length_counter(&mut self) {
        if !self.halt_length_counter && self.length_counter > 0 {
            self.length_counter -= 1;
        }
    }
    
    /// Get the current output volume (0-15)
    pub fn output(&self) -> u8 {
        if self.length_counter == 0 || (self.shift_register & 0x01) != 0 {
            return 0;
        }
        
        if self.constant_volume {
            self.volume
        } else {
            self.envelope_volume
        }
    }
    
    /// Reset the noise channel to its initial state
    pub fn reset(&mut self) {
        self.enabled = false;
        self.length_counter = 0;
        self.envelope_volume = 0;
        self.envelope_decay = 0;
        self.envelope_divider = 0;
        self.envelope_start = false;
        self.constant_volume = false;
        self.volume = 0;
        self.timer = 0;
        self.period_index = 0;  // This was timer_period in the original code
        self.shift_register = 1;  // Initial value is 1
        self.mode_flag = false;   // This was mode in the original code
        self.halt_length_counter = false;
    }
    
    /// Clock the noise channel (called every APU cycle)
    pub fn clock(&mut self) {
        // Decrement the timer
        if self.timer > 0 {
            // Calculate feedback (XOR of bits 0 and 1 for mode 0, or bits 0 and 6 for mode 1)
            let bit0 = self.shift_register & 0x01;
            let bit1 = (self.shift_register >> 1) & 0x01;
            let feedback = (bit0 ^ bit1) & 0x01;
            
            // Shift right and set bit 14 to the feedback value
            self.shift_register = (self.shift_register >> 1) | (feedback << 14);
            
            // Ensure the shift register never becomes zero
            if self.shift_register == 0 {
                self.shift_register = 1;
            }
        } else {
            self.timer -= 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_noise_initialization() {
        let noise = Noise::new();
        assert_eq!(noise.shift_register, 1);
        assert!(!noise.enabled);
        assert_eq!(noise.volume, 0);
    }
    
    #[test]
    fn test_noise_control_register() {
        let mut noise = Noise::new();
        
        // Test control settings
        noise.write_control(0xFF);
        assert!(noise.halt_length_counter);
        assert!(noise.constant_volume);
        assert_eq!(noise.volume, 0x0F);
        
        noise.write_control(0x7F);
        assert!(noise.halt_length_counter);
        assert!(noise.constant_volume);
        assert_eq!(noise.volume, 0x0F);
    }
    
    #[test]
    fn test_noise_period_register() {
        let mut noise = Noise::new();
        
        // Test period settings
        noise.write_period(0x8F);
        assert!(noise.mode_flag);
        assert_eq!(noise.period_index, 0x0F);
    }
    
    #[test]
    fn test_noise_shift_register() {
        let mut noise = Noise::new();
        
        // Test shift register behavior
        noise.shift_register = 0x4000;
        noise.clock_shift_register();
        assert_eq!(noise.shift_register, 0x2000);
        
        // Test feedback in 1-bit mode
        noise.shift_register = 0x0003;
        noise.clock_shift_register();
        assert_eq!(noise.shift_register, 0x8001);
        
        // Test feedback in 6-bit mode
        noise.mode_flag = true;
        noise.shift_register = 0x0041;
        noise.clock_shift_register();
        assert_eq!(noise.shift_register, 0x8020);
    }
    
    #[test]
    fn test_noise_envelope() {
        let mut noise = Noise::new();
        
        // Test envelope decay
        noise.volume = 1;
        noise.envelope_start = true;
        noise.clock_envelope();
        assert_eq!(noise.envelope_volume, 15);
        
        // Clock again to start decay
        noise.clock_envelope();
        assert_eq!(noise.envelope_volume, 14);
    }
}
