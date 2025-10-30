/*!
NES Controller Input Handling
*/

/// Trait for objects that can be reset to their initial state
pub trait Reset {
    /// Reset the object to its initial state
    fn reset(&mut self);
}

/// NES Controller Buttons
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Button {
    A,
    B,
    Select,
    Start,
    Up,
    Down,
    Left,
    Right,
}

/// Represents the state of an NES controller
#[derive(Debug, Default)]
pub struct Controller {
    buttons: u8,
    shift_register: u8,
    strobe: bool,
}

impl Controller {
    /// Create a new controller
    pub fn new() -> Self {
        Controller {
            buttons: 0,
            shift_register: 0,
            strobe: false,
        }
    }

    /// Set the state of a button
    pub fn set_button(&mut self, button: Button, pressed: bool) {
        let mask = 1 << (button as u8);
        if pressed {
            self.buttons |= mask;
        } else {
            self.buttons &= !mask;
        }
    }

    /// Write to the controller (strobe)
    pub fn write(&mut self, value: u8) {
        self.strobe = (value & 0x01) != 0;
        if self.strobe {
            // When strobe is high, continuously load the button states
            self.shift_register = self.buttons;
        }
    }

    /// Read the next button state from the shift register
    pub fn read(&mut self) -> u8 {
        if self.strobe {
            // Always return the state of the A button when strobe is high
            (self.buttons & 0x01) | 0x40
        } else {
            // Read the next bit and shift
            let value = self.shift_register & 0x01;
            self.shift_register = 0x80 | (self.shift_register >> 1);
            value
        }
    }
}

/// Input manager for handling both controllers
#[derive(Debug, Default)]
pub struct InputManager {
    /// Array of two controllers (player 1 and player 2)
    pub controllers: [Controller; 2],
    last_write: u8,
}

impl Reset for InputManager {
    /// Reset the input manager to its initial state
    fn reset(&mut self) {
        for controller in &mut self.controllers {
            controller.buttons = 0;
            controller.shift_register = 0;
            controller.strobe = false;
        }
        self.last_write = 0;
    }
}

impl InputManager {
    /// Create a new input manager
    pub fn new() -> Self {
        InputManager {
            controllers: [Controller::new(), Controller::new()],
            last_write: 0,
        }
    }

    /// Set the state of a button for a specific controller
    pub fn set_button_state(&mut self, controller: usize, button: Button, pressed: bool) {
        if let Some(ctrl) = self.controllers.get_mut(controller) {
            ctrl.set_button(button, pressed);
        }
    }

    /// Handle a write to the controller port
    pub fn write(&mut self, value: u8) {
        self.last_write = value;
        for ctrl in &mut self.controllers {
            ctrl.write(value);
        }
    }

    /// Handle a read from a controller port
    pub fn read(&mut self, port: usize) -> u8 {
        match port {
            0 => self.controllers[0].read(),
            1 => self.controllers[1].read(),
            _ => 0,
        }
    }

    /// Reset the input manager to its initial state
    pub fn reset(&mut self) {
        for controller in &mut self.controllers {
            controller.buttons = 0;
            controller.shift_register = 0;
            controller.strobe = false;
        }
        self.last_write = 0;
    }

    /// Get a mutable reference to a controller
    pub fn get_controller_mut(&mut self, controller: usize) -> Option<&mut Controller> {
        self.controllers.get_mut(controller)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_controller() {
        let mut ctrl = Controller::new();

        // Test button states
        ctrl.set_button(Button::A, true);
        ctrl.set_button(Button::Start, true);

        // Test reading with strobe on
        ctrl.write(1);
        assert_eq!(ctrl.read() & 0x01, 1); // A button

        // Test reading with strobe off
        ctrl.write(0);
        let mut result = 0;
        for _ in 0..8 {
            result = (result >> 1) | ((ctrl.read() & 0x01) << 7);
        }
        assert_eq!(result, 0b10010000); // A and Start buttons
    }
}
