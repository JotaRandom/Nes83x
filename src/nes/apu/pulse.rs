//! NES Pulse (Square) Channel implementation

use super::LENGTH_TABLE;

/// Pulse channel (square wave generator)
#[derive(Debug, Default)]
pub struct Pulse {
    // Channel identification (1 or 2)
    channel: u8,

    // Registers
    duty_cycle: u8, // 2 bits (0-3)
    halt_length_counter: bool,
    constant_volume: bool,
    volume: u8, // 4 bits (0-15)
    sweep_enabled: bool,
    sweep_negate: bool,
    sweep_shift: u8,  // 3 bits (0-7)
    sweep_period: u8, // 3 bits (0-7)
    sweep_reload: bool,
    timer_period: u16, // 11 bits (0-2047)

    // Internal state
    pub enabled: bool,
    pub length_counter: u8,
    pub envelope_volume: u8,
    timer: u16,
    sequencer: u8, // 3 bits (0-7)
    envelope_start: bool,
    envelope_divider: u8,
    envelope_decay: u8,
    sweep_divider: u8,
    sweep_mute: bool,
}

impl Pulse {
    /// Create a new Pulse channel
    pub fn new(channel: u8) -> Self {
        let mut pulse = Pulse {
            channel,
            duty_cycle: 0,
            halt_length_counter: false,
            constant_volume: false,
            volume: 0,
            sweep_enabled: false,
            sweep_negate: false,
            sweep_shift: 0,
            sweep_period: 0,
            sweep_reload: false,
            timer_period: 0,
            enabled: false,
            length_counter: 0,
            envelope_volume: 0,
            timer: 0,
            sequencer: 0,
            envelope_start: false,
            envelope_divider: 0,
            envelope_decay: 0,
            sweep_divider: 0,
            sweep_mute: true,
        };
        pulse.reset();
        pulse
    }

    /// Reset the pulse channel to its initial state
    pub fn reset(&mut self) {
        self.duty_cycle = 0;
        self.halt_length_counter = false;
        self.constant_volume = false;
        self.volume = 0;
        self.sweep_enabled = false;
        self.sweep_negate = false;
        self.sweep_shift = 0;
        self.sweep_period = 0;
        self.sweep_reload = false;
        self.timer_period = 0;
        self.enabled = false;
        self.length_counter = 0;
        self.envelope_volume = 0;
        self.timer = 0;
        self.sequencer = 0;
        self.envelope_start = false;
        self.envelope_divider = 0;
        self.envelope_decay = 0;
        self.sweep_divider = 0;
        self.sweep_mute = true;
    }

    /// Write to control register ($4000 / $4004)
    pub fn write_control(&mut self, value: u8) {
        self.duty_cycle = (value >> 6) & 0x03;
        self.halt_length_counter = (value & 0x20) != 0;
        self.constant_volume = (value & 0x10) != 0;
        self.volume = value & 0x0F;

        if self.halt_length_counter {
            self.length_counter = 0;
        }
    }

    /// Write to sweep register ($4001 / $4005)
    pub fn write_sweep(&mut self, value: u8) {
        self.sweep_enabled = (value & 0x80) != 0;
        self.sweep_period = (value >> 4) & 0x07;
        self.sweep_negate = (value & 0x08) != 0;
        self.sweep_shift = value & 0x07;
        self.sweep_reload = true;

        // Check if sweep unit is muted
        self.update_sweep_mute();
    }

    /// Write to timer low register ($4002 / $4006)
    pub fn write_timer_low(&mut self, value: u8) {
        self.timer_period = (self.timer_period & 0xFF00) | (value as u16);
    }

    /// Write to timer high register ($4003 / $4007)
    pub fn write_timer_high(&mut self, value: u8) {
        self.timer_period = (self.timer_period & 0x00FF) | (((value & 0x07) as u16) << 8);

        // Reload length counter if enabled
        if self.enabled {
            let index = (value >> 3) & 0x1F;
            self.length_counter = LENGTH_TABLE[index as usize];
        }

        // Reset sequencer
        self.sequencer = 0;

        // Restart envelope
        self.envelope_start = true;
    }

    /// Clock the timer
    pub fn clock_timer(&mut self) {
        if self.timer == 0 {
            self.timer = self.timer_period;
            self.sequencer = (self.sequencer + 1) & 0x07;
        } else {
            self.timer -= 1;
        }
    }

    /// Clock the envelope
    pub fn clock_envelope(&mut self) {
        if self.envelope_start {
            self.envelope_volume = 15;
            self.envelope_divider = self.volume;
            self.envelope_start = false;
        } else if self.envelope_divider > 0 {
            self.envelope_divider -= 1;
        } else {
            self.envelope_divider = self.volume;

            if self.envelope_volume > 0 {
                self.envelope_volume -= 1;
            } else if self.halt_length_counter {
                self.envelope_volume = 15;
            }
        }
    }

    /// Clock the sweep unit
    pub fn clock_sweep(&mut self) {
        if self.sweep_reload {
            if self.sweep_divider == 0 && self.sweep_enabled && !self.sweep_mute {
                // Update the period
                self.update_period();
            }
            self.sweep_divider = self.sweep_period;
            self.sweep_reload = false;
        } else if self.sweep_divider > 0 {
            self.sweep_divider -= 1;
        } else {
            if self.sweep_enabled && !self.sweep_mute {
                // Update the period
                self.update_period();
            }
            self.sweep_divider = self.sweep_period;
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
        if self.length_counter == 0
            || self.timer_period < 8
            || self.timer_period > 0x7FF
            || self.sweep_mute
        {
            return 0;
        }

        // Get the current duty cycle value (0-3)
        let duty_pattern = match self.duty_cycle {
            0 => 0b00000001, // 12.5%
            1 => 0b00000011, // 25%
            2 => 0b00001111, // 50%
            3 => 0b11111100, // 25% negated
            _ => 0,
        };

        // Check if the current sequencer step is active in the duty cycle
        let active = (duty_pattern & (1 << self.sequencer)) != 0;

        if active {
            if self.constant_volume {
                self.volume
            } else {
                self.envelope_volume
            }
        } else {
            0
        }
    }

    /// Clock the pulse channel (called every APU cycle)
    pub fn clock(&mut self) {
        self.clock_timer();
        self.clock_envelope();
        self.clock_sweep();

        // Update the output
        self.output();

        // Update the sweep mute flag
        self.update_sweep_mute();
    }

    /// Update the period based on the sweep unit
    fn update_period(&mut self) {
        if self.sweep_shift > 0 {
            let shift = self.sweep_shift as u16;
            let change = self.timer_period >> shift;

            if self.sweep_negate {
                // Invert the change for channel 1 (pulse 1)
                if self.channel == 1 {
                    self.timer_period = self.timer_period.wrapping_sub(change).wrapping_sub(1);
                } else {
                    self.timer_period = self.timer_period.wrapping_sub(change);
                }
            } else {
                self.timer_period = self.timer_period.wrapping_add(change);
            }

            // Check if the period is invalid
            if self.timer_period < 8 || self.timer_period > 0x7FF {
                self.sweep_mute = true;
            }
        }
    }

    /// Perform a sweep operation
    fn sweep(&mut self) {
        if !self.sweep_enabled || self.sweep_shift == 0 {
            return;
        }

        let change = self.timer_period >> self.sweep_shift;

        if self.sweep_negate {
            // Invert the change for channel 1 (pulse 1)
            if self.channel == 1 {
                self.timer_period = self.timer_period.wrapping_sub(change).wrapping_sub(1);
            } else {
                self.timer_period = self.timer_period.wrapping_sub(change);
            }
        } else {
            self.timer_period = self.timer_period.wrapping_add(change);
        }

        // Update mute flag
        self.update_sweep_mute();
    }

    /// Update the sweep mute flag
    fn update_sweep_mute(&mut self) {
        self.sweep_mute = self.timer_period < 8 || self.timer_period > 0x7FF;
    }
}

// Duty cycle patterns (4 steps each)
const DUTY_CYCLES: [u8; 4] = [
    0b01000000, // 12.5% (0 0 0 0 0 0 0 1)
    0b01100000, // 25%   (0 0 0 0 0 1 1 1)
    0b01111000, // 50%   (0 0 1 1 1 1 1 1)
    0b10011111, // 25% negated (1 1 1 1 1 0 0 1)
];

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pulse_initialization() {
        let pulse = Pulse::new(1);
        assert_eq!(pulse.channel, 1);
        assert_eq!(pulse.duty_cycle, 0);
        assert_eq!(pulse.volume, 0);
        assert!(!pulse.enabled);
    }

    #[test]
    fn test_pulse_control_register() {
        let mut pulse = Pulse::new(1);

        // Test duty cycle and volume settings
        pulse.write_control(0x7F);
        assert_eq!(pulse.duty_cycle, 1);
        assert!(pulse.halt_length_counter);
        assert!(pulse.constant_volume);
        assert_eq!(pulse.volume, 0x0F);
    }

    #[test]
    fn test_pulse_sweep_register() {
        let mut pulse = Pulse::new(1);

        // Test sweep settings
        pulse.write_sweep(0x88);
        assert!(pulse.sweep_enabled);
        assert_eq!(pulse.sweep_period, 1);
        assert!(pulse.sweep_negate);
        assert_eq!(pulse.sweep_shift, 0);
    }

    #[test]
    fn test_pulse_timer() {
        let mut pulse = Pulse::new(1);

        // Set timer period
        pulse.write_timer_low(0x34);
        pulse.write_timer_high(0x12);
        assert_eq!(pulse.timer_period, 0x1234 & 0x7FF);

        // Test timer counting
        let initial_timer = pulse.timer;
        pulse.clock_timer();
        assert_eq!(pulse.timer, initial_timer.wrapping_sub(1) & 0x7FF);
    }
}
