//! NES APU Frame Counter implementation

/// Frame counter for the NES APU
/// Handles the timing of the APU's frame counter and IRQ generation
#[derive(Debug, Default)]
pub struct FrameCounter {
    /// Current step in the sequence (0-3 for 4-step, 0-4 for 5-step)
    pub step: u8,

    /// False = 4-step sequence (default), True = 5-step sequence
    pub mode: bool,

    /// If true, disable frame interrupt
    pub irq_inhibit: bool,
}

impl FrameCounter {
    /// Create a new FrameCounter in its default state
    pub fn new() -> Self {
        FrameCounter {
            step: 0,
            mode: false, // 4-step mode by default
            irq_inhibit: false,
        }
    }

    /// Reset the frame counter to its initial state
    pub fn reset(&mut self) {
        self.step = 0;
        self.mode = false;
        self.irq_inhibit = false;
    }

    /// Get the current step in the sequence
    pub fn current_step(&self) -> u8 {
        self.step
    }

    /// Check if in 5-step mode
    pub fn is_five_step_mode(&self) -> bool {
        self.mode
    }

    /// Check if IRQ is inhibited
    pub fn is_irq_inhibited(&self) -> bool {
        self.irq_inhibit
    }

    /// Write to the frame counter register ($4017)
    ///
    /// Bit 7: 0 = 4-step sequence, 1 = 5-step sequence
    /// Bit 6: 0 = IRQ enabled, 1 = IRQ disabled
    pub fn write(&mut self, value: u8) {
        self.mode = (value & 0x80) != 0; // Bit 7: Mode
        self.irq_inhibit = (value & 0x40) != 0; // Bit 6: IRQ inhibit

        // If mode is set (5-step sequence), reset the step counter
        // and immediately clock the envelope and linear counter
        if self.mode {
            self.step = 0;
            // TODO: Trigger quarter frame and half frame clocks immediately
        }

        // If IRQ is disabled, clear any pending IRQ
        if self.irq_inhibit {
            // TODO: Clear any pending frame IRQ
        }
    }

    /// Clock the frame counter (called every APU cycle)
    /// Returns a tuple of (quarter_frame, half_frame) indicating which clocks to trigger
    pub fn clock(&mut self) -> (bool, bool) {
        let result = self.step();
        result
    }

    /// Advance the frame counter by one CPU cycle (1.79MHz / 2)
    /// Returns a tuple of (quarter_frame, half_frame) indicating which clocks to trigger
    pub fn step(&mut self) -> (bool, bool) {
        let mut quarter_frame = false;
        let mut half_frame = false;

        if self.mode {
            // 5-step sequence (mode 1)
            match self.step {
                0 | 2 => {
                    // Quarter frame on steps 0 and 2
                    quarter_frame = true;
                }
                1 | 3 => {
                    // Half frame on steps 1 and 3
                    quarter_frame = true;
                    half_frame = true;
                }
                _ => {}
            }

            // Wrap around after 5 steps (0-4)
            self.step = if self.step >= 4 { 0 } else { self.step + 1 };
        } else {
            // 4-step sequence (mode 0)
            match self.step {
                0 | 2 => {
                    // Quarter frame on steps 0 and 2
                    quarter_frame = true;
                }
                1 => {
                    // Half frame on step 1
                    quarter_frame = true;
                    half_frame = true;
                }
                3 => {
                    // Half frame on step 3
                    quarter_frame = true;
                    half_frame = true;

                    // Generate IRQ at end of 4-step sequence if not inhibited
                    if !self.irq_inhibit {
                        // TODO: Trigger IRQ
                    }
                }
                _ => {}
            }

            // Wrap around after 4 steps (0-3)
            self.step = if self.step >= 3 { 0 } else { self.step + 1 };
        }

        (quarter_frame, half_frame)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_frame_counter_initialization() {
        let fc = FrameCounter::new();
        assert_eq!(fc.step, 0);
        assert!(!fc.mode);
        assert!(!fc.irq_inhibit);
    }

    #[test]
    fn test_frame_counter_reset() {
        let mut fc = FrameCounter {
            step: 3,
            mode: true,
            irq_inhibit: true,
        };

        fc.reset();

        assert_eq!(fc.step, 0);
        assert!(!fc.mode);
        assert!(!fc.irq_inhibit);
    }
}
