//! Emulator frontend with minifb UI

use minifb::{Key, Window, WindowOptions};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use thiserror::Error;
use log::{info, error};

use crate::nes::{input::Button as NesButton, Nes, NesError};

/// Errors that can occur in the emulator frontend
#[derive(Error, Debug)]
pub enum EmulatorError {
    #[error("minifb error: {0}")]
    MinifbError(String),

    #[error("NES error: {0}")]
    NesError(#[from] NesError),

    #[error("I/O error: {0}")]
    IoError(#[from] std::io::Error),
}

/// The emulator application
pub struct Emulator {
    /// The NES system
    nes: Arc<Mutex<Nes>>,

    /// The minifb window
    window: Option<Window>,

    /// The current ROM path, if any
    pub rom_path: Option<PathBuf>,

    /// Whether the emulator is running
    is_running: bool,
}

impl Emulator {
    /// Create a new emulator instance
    pub fn new() -> Result<Self, EmulatorError> {
        // Create the NES system
        let nes = Arc::new(Mutex::new(Nes::new()));

        Ok(Emulator {
            nes,
            window: None,
            rom_path: None,
            is_running: false,
        })
    }

    /// Initialize the emulator UI
    pub fn init(&mut self) -> Result<(), EmulatorError> {
        // Create the window
        let mut window = Window::new(
            "Nes83x - NES Emulator",
            256 * 2,
            240 * 2,
            WindowOptions::default(),
        )
        .map_err(|e| EmulatorError::MinifbError(e.to_string()))?;

        // Limit to max ~60 fps
        window.limit_update_rate(Some(std::time::Duration::from_micros(16600)));

        self.window = Some(window);

        Ok(())
    }

    /// Load a ROM file
    pub fn load_rom<P: Into<PathBuf>>(&mut self, path: P) -> Result<(), EmulatorError> {
        let path = path.into();

        info!("Loading ROM in minifb emulator: {}", path.display());

        // Try to load the ROM
        {
            let mut nes = self.nes.lock().unwrap();
            nes.load_rom(&path)?;
            info!("ROM loaded, NES is_running: {}", nes.is_running);
        }

        // Store the ROM path
        self.rom_path = Some(path);

        // Reset the NES
        self.reset()?;

        // Set NES to running after reset
        {
            let mut nes = self.nes.lock().unwrap();
            nes.is_running = true;
        }

        // Start the emulation
        if !self.is_running {
            self.start()?;
        }

        eprintln!("ROM loaded and emulation started");

        Ok(())
    }

    /// Start the emulation
    pub fn start(&mut self) -> Result<(), EmulatorError> {
        if self.is_running {
            return Ok(());
        }

        self.is_running = true;
        eprintln!("Emulation started");

        Ok(())
    }

    /// Stop the emulation
    pub fn stop(&mut self) -> Result<(), EmulatorError> {
        self.is_running = false;
        Ok(())
    }

    /// Reset the emulator
    pub fn reset(&mut self) -> Result<(), EmulatorError> {
        self.nes.lock().unwrap().reset()?;
        Ok(())
    }

    /// Run the emulator's main loop
    pub fn run(&mut self) -> Result<(), EmulatorError> {
        eprintln!("About to run minifb app");

        if let Some(ref mut window) = self.window {
            // Create a buffer for the window
            let mut buffer: Vec<u32> = vec![0; 256 * 240];

            while window.is_open() && !window.is_key_down(Key::Escape) {
                // Handle input
                Emulator::handle_input(&self.nes, window);

                // Run a frame if running
                if self.is_running {
                    let mut nes = self.nes.lock().unwrap();
                    if nes.is_running {
                        eprintln!("Running frame");
                        let _ = nes.run_frame();

                        // Copy framebuffer to buffer
                        let ppu = nes.ppu.borrow();
                        let framebuffer = &ppu.framebuffer;

                        // Convert ARGB32 to RGBA32 for minifb (swap R and B)
                        for (i, pixel) in framebuffer.chunks(4).enumerate() {
                            let a = pixel[0] as u32;
                            let r = pixel[1] as u32;
                            let g = pixel[2] as u32;
                            let b = pixel[3] as u32;
                            buffer[i] = (r << 24) | (g << 16) | (b << 8) | a;
                        }
                    }
                }

                // Update the window
                window.update_with_buffer(&buffer, 256, 240)
                    .map_err(|e| EmulatorError::MinifbError(e.to_string()))?;
            }
        }

        eprintln!("minifb app finished");
        Ok(())
    }

    /// Handle input from the window
    fn handle_input(nes: &Arc<Mutex<Nes>>, window: &Window) {
        let mut nes = nes.lock().unwrap();

        // Map keys to NES controller buttons
        nes.set_button_state(0, NesButton::Left, window.is_key_down(Key::Left));
        nes.set_button_state(0, NesButton::Right, window.is_key_down(Key::Right));
        nes.set_button_state(0, NesButton::Up, window.is_key_down(Key::Up));
        nes.set_button_state(0, NesButton::Down, window.is_key_down(Key::Down));
        nes.set_button_state(0, NesButton::A, window.is_key_down(Key::Z));
        nes.set_button_state(0, NesButton::B, window.is_key_down(Key::X));
        nes.set_button_state(0, NesButton::Start, window.is_key_down(Key::Enter));
        nes.set_button_state(0, NesButton::Select, window.is_key_down(Key::LeftShift) || window.is_key_down(Key::RightShift));
    }

    /// Clean up resources
    pub fn cleanup(&mut self) -> Result<(), EmulatorError> {
        self.stop()?;
        Ok(())
    }
}

impl Drop for Emulator {
    fn drop(&mut self) {
        let _ = self.cleanup();
    }
}