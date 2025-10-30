//! NES Audio Processing Unit (APU) emulation

// Constants
/// Length counter lookup table (0-31)
pub const LENGTH_TABLE: [u8; 32] = [
    10, 254, 20, 2, 40, 4, 80, 6, 160, 8, 60, 10, 14, 12, 26, 14,
    12, 16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30
];

/// Noise period lookup table
pub const NOISE_PERIOD_TABLE: [u16; 16] = [
    4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
];

/// DMC rate table (NTSC)
pub const DMC_RATE_TABLE: [u16; 16] = [
    428, 380, 340, 320, 286, 254, 226, 214, 190, 160, 142, 128, 106, 84, 72, 54
];

mod pulse;
mod triangle;
mod noise;
mod dmc;
mod frame_counter;

use crate::nes::Memory;

// Re-export the main APU types
pub use self::pulse::Pulse;
pub use self::triangle::Triangle;
pub use self::noise::Noise;
pub use self::dmc::Dmc;
pub use self::frame_counter::FrameCounter;

/// The NES APU (Audio Processing Unit)
#[derive(Debug)]
pub struct Apu {
    // Pulse channels (square waves)
    pulse1: Pulse,
    pulse2: Pulse,
    
    // Triangle channel
    triangle: Triangle,
    
    // Noise channel
    noise: Noise,
    
    // DMC channel (Delta Modulation Channel)
    dmc: Dmc,
    
    // Frame counter
    frame_counter: FrameCounter,
    
    // Internal state
    cycle: u64,
    frame_irq: bool,
    dmc_irq: bool,
}

impl Apu {
    /// Create a new APU instance
    pub fn new() -> Self {
        Apu {
            pulse1: Pulse::new(1),
            pulse2: Pulse::new(2),
            triangle: Triangle::new(),
            noise: Noise::new(),
            dmc: Dmc::new(),
            frame_counter: FrameCounter::new(),
            cycle: 0,
            frame_irq: false,
            dmc_irq: false,
        }
    }
    
    /// Reset the APU to its initial state
    pub fn reset(&mut self) {
        self.pulse1 = Pulse::new(1);
        self.pulse2 = Pulse::new(2);
        self.triangle = Triangle::new();
        self.noise = Noise::new();
        self.dmc = Dmc::new();
        self.frame_counter = FrameCounter::new();
        self.cycle = 0;
        self.frame_irq = false;
        self.dmc_irq = false;
    }
    
    /// Execute one APU cycle (1 CPU cycle = 2 APU cycles)
    pub fn tick(&mut self, memory: &impl Memory) -> u8 {
        // Clock the frame counter and audio channels every other CPU cycle (APU runs at CPU rate / 2)
        if self.cycle % 2 == 0 {
            self.clock_frame_counter();
        }
        
        // Clock the DMC channel (runs at CPU rate)
        let _dmc_output = if self.dmc.enabled {
            self.dmc.clock(memory)
        } else {
            0
        };
        
        // Generate and return the current audio sample
        let sample = self.generate_sample();
        
        // Update cycle counter
        self.cycle = self.cycle.wrapping_add(1);
        
        sample
    }
    
    /// Clock the frame counter and APU channels
    fn clock_frame_counter(&mut self) {
        // Clock the frame counter and get frame timing events
        let (quarter_frame, half_frame) = self.frame_counter.clock();
        
        // Clock each audio channel
        self.pulse1.clock();
        self.pulse2.clock();
        self.triangle.clock();
        self.noise.clock();
        
        // Handle frame counter events
        if quarter_frame {
            self.quarter_frame();
        }
        if half_frame {
            self.half_frame();
        }
    }
    
    /// Step the frame counter (kept for backward compatibility)
    fn step_frame_counter(&mut self) {
        // This method is kept for backward compatibility but is no longer used
        // The frame counter is now handled in clock_frame_counter()
    }
    
    /// Quarter frame clock (envelopes, triangle's linear counter)
    fn quarter_frame(&mut self) {
        self.pulse1.clock_envelope();
        self.pulse2.clock_envelope();
        self.noise.clock_envelope();
        self.triangle.clock_linear_counter();
    }
    
    /// Half frame clock (length counters, sweep)
    fn half_frame(&mut self) {
        self.pulse1.clock_sweep();
        self.pulse2.clock_sweep();
        self.pulse1.clock_length_counter();
        self.pulse2.clock_length_counter();
        self.triangle.clock_length_counter();
        self.noise.clock_length_counter();
    }
    
    /// Generate audio samples
    pub fn generate_sample(&self) -> u8 {
        // Mix all channels
        let pulse1 = self.pulse1.output();
        let pulse2 = self.pulse2.output();
        let triangle = self.triangle.output();
        let noise = self.noise.output();
        let dmc = self.dmc.output(); // Use the output method
        
        // Simple mixing (can be improved with proper NES audio mixing)
        let pulse_out = 0.00752 * (pulse1 + pulse2) as f32;
        let tnd_out = 0.00851 * triangle as f32 + 
                     0.00494 * noise as f32 + 
                     0.00335 * dmc as f32;
        
        let sample = (pulse_out + tnd_out).clamp(0.0, 1.0);
        (sample * 255.0) as u8
    }
    
    /// Write to an APU register
    pub fn write_register(&mut self, addr: u16, value: u8) {
        match addr {
            // Pulse 1
            0x4000 => self.pulse1.write_control(value),
            0x4001 => self.pulse1.write_sweep(value),
            0x4002 => self.pulse1.write_timer_low(value),
            0x4003 => self.pulse1.write_timer_high(value),
            
            // Pulse 2
            0x4004 => self.pulse2.write_control(value),
            0x4005 => self.pulse2.write_sweep(value),
            0x4006 => self.pulse2.write_timer_low(value),
            0x4007 => self.pulse2.write_timer_high(value),
            
            // Triangle
            0x4008 => self.triangle.write_control(value),
            0x400A => self.triangle.write_timer_low(value),
            0x400B => self.triangle.write_timer_high(value),
            
            // Noise
            0x400C => self.noise.write_control(value),
            0x400E => self.noise.write_period(value),
            0x400F => self.noise.write_length_counter(value),
            
            // DMC
            0x4010 => self.dmc.write_control(value),
            0x4011 => self.dmc.write_value(value),
            0x4012 => self.dmc.write_address(value),
            0x4013 => self.dmc.write_length(value),
            
            // Status/Frame Counter
            0x4015 => self.write_status(value),
            0x4017 => frame_counter::FrameCounter::write(&mut self.frame_counter, value),
            
            _ => {}
        }
    }
    
    /// Write to status register ($4015)
    fn write_status(&mut self, value: u8) {
        self.pulse1.enabled = (value & 0x01) != 0;
        self.pulse2.enabled = (value & 0x02) != 0;
        self.triangle.enabled = (value & 0x04) != 0;
        self.noise.enabled = (value & 0x08) != 0;
        self.dmc.enabled = (value & 0x10) != 0;
        
        // If a channel is disabled, its length counter is forced to 0
        if !self.pulse1.enabled {
            self.pulse1.length_counter = 0;
        }
        if !self.pulse2.enabled {
            self.pulse2.length_counter = 0;
        }
        if !self.triangle.enabled {
            self.triangle.length_counter = 0;
        }
        if !self.noise.enabled {
            self.noise.length_counter = 0;
        }
    }
}

impl Memory for Apu {
    fn read_byte(&self, addr: u16) -> std::io::Result<u8> {
        match addr {
            // Status register ($4015)
            0x4015 => {
                let mut result = 0;
                
                if self.pulse1.length_counter > 0 {
                    result |= 0x01;
                }
                if self.pulse2.length_counter > 0 {
                    result |= 0x02;
                }
                if self.triangle.length_counter > 0 {
                    result |= 0x04;
                }
                if self.noise.length_counter > 0 {
                    result |= 0x08;
                }
                if self.dmc.bytes_remaining > 0 {
                    result |= 0x10;
                }
                if self.frame_irq {
                    result |= 0x40;
                }
                if self.dmc_irq {
                    result |= 0x80;
                }
                
                // Reading the status register clears the frame interrupt flag
                if let Some(frame_irq) = &mut self.frame_irq {
                    *frame_irq = false;
                }
                
                Ok(result)
            }
            _ => Ok(0),
        }
    }
    
    fn write_byte(&mut self, addr: u16, value: u8) -> std::io::Result<()> {
        self.write_register(addr, value);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nes::Memory;
    
    struct TestMemory {
        data: [u8; 0x10000],
    }
    
    impl TestMemory {
        fn new() -> Self {
            TestMemory {
                data: [0; 0x10000],
            }
        }
        
        fn write(&self, addr: u16, value: u8) {
            self.data[addr as usize] = value;
        }
    }
    
    impl Memory for TestMemory {
        fn read_byte(&self, addr: u16) -> std::io::Result<u8> {
            Ok(self.data[addr as usize])
        }
        
        fn write_byte(&mut self, _addr: u16, _value: u8) -> std::io::Result<()> {
            Ok(())
        }
    }
    
    #[test]
    fn test_apu_reset() {
        let apu = Apu::new();
        assert_eq!(apu.cycle, 0);
        assert!(!apu.frame_irq);
        assert!(!apu.dmc_irq);
    }
    
    #[test]
    fn test_apu_register_write() {
        let mut apu = Apu::new();
        
        // Test pulse 1 control register
        apu.write_register(0x4000, 0x3F); // Max volume, constant volume
        assert_eq!(apu.pulse1.envelope_volume, 15);
        
        // Test frame counter
        apu.write_register(0x4017, 0x40); // Set frame counter mode 0, IRQ inhibit
        assert_eq!(apu.frame_counter.mode, 0);
        assert!(apu.frame_counter.irq_inhibit);
    }
}
