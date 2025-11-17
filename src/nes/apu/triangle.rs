//! NES Triangle Channel implementation

use super::LENGTH_TABLE;

/// Triangle channel (triangle wave generator)
#[derive(Debug, Default)]
pub struct Triangle {
    // Registers
    control_flag: bool, // Length counter halt / linear counter control
    linear_counter_reload: bool,
    linear_counter_reload_value: u8, // 7 bits (0-127)
    timer_period: u16,               // 11 bits (0-2047)
    pub length_counter: u8,

    // Internal state
    pub enabled: bool,
    linear_counter: u8,
    timer: u16,
    sequencer_step: u8, // 5 bits (0-31)
    sequencer_reload: bool,
}

impl Triangle {
    /// Create a new Triangle channel
    pub fn new() -> Self {
        let mut triangle = Triangle {
            control_flag: false,
            linear_counter_reload: false,
            linear_counter_reload_value: 0,
            timer_period: 0,
            length_counter: 0,
            enabled: false,
            linear_counter: 0,
            timer: 0,
            sequencer_step: 0,
            sequencer_reload: false,
        };
        triangle.reset();
        triangle
    }

    /// Reset the triangle channel to its initial state
    pub fn reset(&mut self) {
        self.control_flag = false;
        self.linear_counter_reload = false;
        self.linear_counter_reload_value = 0;
        self.timer_period = 0;
        self.length_counter = 0;
        self.enabled = false;
        self.linear_counter = 0;
        self.timer = 0;
        self.sequencer_step = 0;
        self.sequencer_reload = false;
    }

    /// Write to control register ($4008)
    pub fn write_control(&mut self, value: u8) {
        self.control_flag = (value & 0x80) != 0;
        self.linear_counter_reload_value = value & 0x7F;

        // If the control flag is set, the length counter is also halted
        if self.control_flag {
            self.length_counter = 0;
        }
    }

    /// Write to timer low register ($400A)
    pub fn write_timer_low(&mut self, value: u8) {
        self.timer_period = (self.timer_period & 0xFF00) | (value as u16);
    }

    /// Write to timer high register ($400B)
    pub fn write_timer_high(&mut self, value: u8) {
        self.timer_period = (self.timer_period & 0x00FF) | (((value & 0x07) as u16) << 8);

        // Reload length counter if enabled
        if self.enabled {
            let index = (value >> 3) & 0x1F;
            self.length_counter = LENGTH_TABLE[index as usize];
        }

        // Set the linear counter reload flag
        self.linear_counter_reload = true;
    }

    /// Clock the timer
    pub fn clock_timer(&mut self) {
        if self.timer == 0 {
            self.timer = self.timer_period;

            // Only update the sequencer if the linear counter and length counter are not zero
            if self.linear_counter > 0 && self.length_counter > 0 && !self.sequencer_reload {
                self.sequencer_step = (self.sequencer_step + 1) & 0x1F;
            }
        } else {
            self.timer -= 1;
        }
    }

    /// Clock the linear counter
    pub fn clock_linear_counter(&mut self) {
        if self.linear_counter_reload {
            self.linear_counter = self.linear_counter_reload_value;
        } else if self.linear_counter > 0 {
            self.linear_counter -= 1;
        }

        if !self.control_flag {
            self.linear_counter_reload = false;
        }
    }

    /// Clock the length counter
    pub fn clock_length_counter(&mut self) {
        if !self.control_flag && self.length_counter > 0 {
            self.length_counter -= 1;
        }
    }

    /// Get the current output volume (0-15)
    pub fn output(&self) -> u8 {
        // The triangle channel outputs a 4-bit value (0-15)
        // based on the current sequencer step
        [
            0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6,
            5, 4, 3, 2, 1, 0, 0,
        ][self.sequencer_step as usize]
    }

    /// Clock the triangle channel (called every APU cycle)
    pub fn clock(&mut self) {
        if self.linear_counter > 0 && self.length_counter > 0 && self.timer_period >= 2 {
            // Only clock the sequencer if the linear counter and length counter are non-zero
            // and the timer period is at least 2 (minimum period for the triangle channel)
            if self.timer == 0 {
                self.timer = self.timer_period;

                // Clock the sequencer
                if self.sequencer_reload || self.sequencer_step == 0x1F {
                    self.sequencer_step = 0;
                    self.sequencer_reload = false;
                } else if self.sequencer_step < 0x10 {
                    self.sequencer_step += 1;
                } else {
                    self.sequencer_step = 0x1F - (self.sequencer_step - 0x10);
                }
            } else {
                self.timer -= 1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_triangle_initialization() {
        let triangle = Triangle::new();
        assert!(!triangle.control_flag);
        assert_eq!(triangle.linear_counter_reload_value, 0);
        assert_eq!(triangle.timer_period, 0);
        assert!(!triangle.enabled);
    }

    #[test]
    fn test_triangle_control_register() {
        let mut triangle = Triangle::new();

        // Test control flag and linear counter reload value
        triangle.write_control(0xFF);
        assert!(triangle.control_flag);
        assert_eq!(triangle.linear_counter_reload_value, 0x7F);

        triangle.write_control(0x7F);
        assert!(!triangle.control_flag);
        assert_eq!(triangle.linear_counter_reload_value, 0x7F);
    }

    #[test]
    fn test_triangle_timer() {
        let mut triangle = Triangle::new();

        // Set timer period
        triangle.write_timer_low(0x34);
        triangle.write_timer_high(0x12);
        assert_eq!(triangle.timer_period, 0x1234 & 0x7FF);

        // Test timer counting
        let initial_timer = triangle.timer;
        triangle.clock_timer();
        assert_eq!(triangle.timer, initial_timer.wrapping_sub(1) & 0x7FF);
    }

    #[test]
    fn test_triangle_linear_counter() {
        let mut triangle = Triangle::new();

        // Set linear counter reload value
        triangle.write_control(0x1F);

        // Clock the linear counter with reload flag set
        triangle.linear_counter_reload = true;
        triangle.clock_linear_counter();
        assert_eq!(triangle.linear_counter, 0x1F);

        // Clock again without reload flag
        triangle.linear_counter_reload = false;
        triangle.clock_linear_counter();
        assert_eq!(triangle.linear_counter, 0x1E);
    }

    #[test]
    fn test_triangle_sequencer() {
        let mut triangle = Triangle::new();

        // Enable the channel and set up the sequencer
        triangle.enabled = true;
        triangle.linear_counter = 1;
        triangle.length_counter = 1;
        triangle.timer_period = 1;

        // Step through the sequencer
        for _ in 0..32 {
            triangle.clock_timer();
        }

        // The sequencer should have gone through all 32 steps
        assert_eq!(triangle.sequencer_step, 0);
    }
}
