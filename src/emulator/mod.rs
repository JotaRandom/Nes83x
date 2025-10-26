//! Emulator frontend with GTK4 UI

use gtk::prelude::*;
use gtk::{Application, ApplicationWindow, DrawingArea};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use thiserror::Error;

use crate::nes::{Nes, NesError};

/// Errors that can occur in the emulator frontend
#[derive(Error, Debug)]
pub enum EmulatorError {
    #[error("GTK error: {0}")]
    GtkError(String),
    
    #[error("NES error: {0}")]
    NesError(#[from] NesError),
    
    #[error("I/O error: {0}")]
    IoError(#[from] std::io::Error),
}

/// The emulator application
pub struct Emulator {
    /// The NES system
    nes: Arc<Mutex<Nes>>,
    
    /// The GTK application
    app: Application,
    
    /// The main window
    window: Option<ApplicationWindow>,
    
    /// The drawing area for the NES screen
    drawing_area: Option<DrawingArea>,
    
    /// The current ROM path, if any
    rom_path: Option<PathBuf>,
    
    /// Whether the emulator is running
    is_running: bool,
}

impl Emulator {
    /// Create a new emulator instance
    pub fn new() -> Result<Self, EmulatorError> {
        // Initialize GTK
        if gtk::init().is_err() {
            return Err(EmulatorError::GtkError("Failed to initialize GTK".into()));
        }
        
        // Create the GTK application
        let app = Application::builder()
            .application_id("com.nes83x.emulator")
            .build()
            .map_err(|e| EmulatorError::GtkError(e.to_string()))?;
        
        // Create the NES system
        let nes = Arc::new(Mutex::new(Nes::new()));
        
        Ok(Emulator {
            nes,
            app,
            window: None,
            drawing_area: None,
            rom_path: None,
            is_running: false,
        })
    }
    
    /// Initialize the emulator UI
    pub fn init(&mut self) -> Result<(), EmulatorError> {
        // Create the main window
        let window = ApplicationWindow::builder()
            .title("Nes83x - NES Emulator")
            .default_width(256 * 2)  // 2x scale
            .default_height(240 * 2) // 2x scale
            .build();
        
        // Create the drawing area
        let drawing_area = DrawingArea::new();
        
        // Set up the drawing area
        drawing_area.set_draw_func(|_, cr, width, height| {
            // Clear the background
            cr.set_source_rgb(0.1, 0.1, 0.1);
            cr.paint().expect("Failed to paint background");
            
            // Draw a placeholder
            cr.set_source_rgb(0.5, 0.5, 0.5);
            cr.rectangle(0.0, 0.0, width as f64, height as f64);
            cr.stroke().expect("Failed to draw placeholder");
            
            cr.set_source_rgb(1.0, 1.0, 1.0);
            cr.select_font_face("Sans", gtk::cairo::FontSlant::Normal, gtk::cairo::FontWeight::Normal);
            cr.set_font_size(24.0);
            
            let text = "NES Emulator - Load a ROM to start";
            let extents = cr.text_extents(text).expect("Failed to get text extents");
            let x = (width as f64 - extents.width()) / 2.0 - extents.x_bearing();
            let y = (height as f64 - extents.height()) / 2.0 - extents.y_bearing();
            
            cr.move_to(x, y);
            cr.show_text(text).expect("Failed to show text");
        });
        
        // Connect key events
        let nes_clone = Arc::clone(&self.nes);
        drawing_area.set_key_press_handler(Some(Box::new(move |keyval, _, _| {
            let mut nes = nes_clone.lock().unwrap();
            
            // Map keys to NES controller buttons
            // This is a simple mapping, you might want to make this configurable
            match keyval {
                // Player 1
                gtk::gdk::Key::Left => nes.set_button_state(0, 6, true),  // Left
                gtk::gdk::Key::Right => nes.set_button_state(0, 7, true), // Right
                gtk::gdk::Key::Up => nes.set_button_state(0, 4, true),    // Up
                gtk::gdk::Key::Down => nes.set_button_state(0, 5, true),  // Down
                gtk::gdk::Key::z => nes.set_button_state(0, 0, true),     // A
                gtk::gdk::Key::x => nes.set_button_state(0, 1, true),     // B
                gtk::gdk::Key::Return => nes.set_button_state(0, 2, true), // Start
                gtk::gdk::Key::Shift_L | gtk::gdk::Key::Shift_R => nes.set_button_state(0, 3, true), // Select
                gtk::gdk::Key::Escape => {
                    // Quit on Escape
                    let app = window.application().unwrap();
                    app.quit();
                }
                _ => {}
            }
            
            gtk::Inhibit(false)
        })));
        
        // Handle key release events
        let nes_clone = Arc::clone(&self.nes);
        drawing_area.set_key_release_handler(Some(Box::new(move |keyval, _, _| {
            let mut nes = nes_clone.lock().unwrap();
            
            // Map keys to NES controller buttons
            match keyval {
                // Player 1
                gtk::gdk::Key::Left => nes.set_button_state(0, 6, false),  // Left
                gtk::gdk::Key::Right => nes.set_button_state(0, 7, false), // Right
                gtk::gdk::Key::Up => nes.set_button_state(0, 4, false),    // Up
                gtk::gdk::Key::Down => nes.set_button_state(0, 5, false),  // Down
                gtk::gdk::Key::z => nes.set_button_state(0, 0, false),     // A
                gtk::gdk::Key::x => nes.set_button_state(0, 1, false),     // B
                gtk::gdk::Key::Return => nes.set_button_state(0, 2, false), // Start
                gtk::gdk::Key::Shift_L | gtk::gdk::Key::Shift_R => nes.set_button_state(0, 3, false), // Select
                _ => {}
            }
            
            gtk::Inhibit(false)
        })));
        
        // Add the drawing area to the window
        window.set_child(Some(&drawing_area));
        
        // Connect the window's close request
        let app = self.app.clone();
        window.connect_close_request(move |_| {
            app.quit();
            gtk::Inhibit(false)
        });
        
        // Show the window
        window.present();
        
        // Store references to the window and drawing area
        self.window = Some(window);
        self.drawing_area = Some(drawing_area);
        
        Ok(())
    }
    
    /// Load a ROM file
    pub fn load_rom<P: Into<PathBuf>>(&mut self, path: P) -> Result<(), EmulatorError> {
        let path = path.into();
        
        // Try to load the ROM
        self.nes.lock().unwrap().load_rom(&path)?;
        
        // Store the ROM path
        self.rom_path = Some(path);
        
        // Reset the NES
        self.reset()?;
        
        // Start the emulation thread if not already running
        if !self.is_running {
            self.start()?;
        }
        
        Ok(())
    }
    
    /// Start the emulation
    pub fn start(&mut self) -> Result<(), EmulatorError> {
        if self.is_running {
            return Ok(());
        }
        
        self.is_running = true;
        
        // In a real implementation, you would start an emulation thread here
        // that runs the NES at 60 FPS and updates the screen
        
        Ok(())
    }
    
    /// Stop the emulation
    pub fn stop(&mut self) -> Result<(), EmulatorError> {
        self.is_running = false;
        // In a real implementation, you would stop the emulation thread here
        Ok(())
    }
    
    /// Reset the emulator
    pub fn reset(&mut self) -> Result<(), EmulatorError> {
        self.nes.lock().unwrap().reset()?;
        Ok(())
    }
    
    /// Run the emulator's main loop
    pub fn run(&self) -> Result<(), EmulatorError> {
        self.app.run();
        Ok(())
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
