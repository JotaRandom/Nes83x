//! Emulator frontend with GTK4 UI

// GTK4 imports
use gtk::prelude::*;
use gtk::{Application, ApplicationWindow, DrawingArea};
use gtk::gdk::Key;
use gtk::glib;
use gtk::glib::Propagate;

// Re-export Inhibit for convenience
use gtk::gdk::Inhibit;

// GDK4 imports for key handling
use gdk4 as gdk;
use gdk::prelude::*;
use gtk::EventControllerKey;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use thiserror::Error;

use crate::nes::{Nes, NesError, input::Button};

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
        // Create the GTK application
        let app = Application::builder()
            .application_id("com.nes83x.emulator")
            .build();

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

        // Create a key controller for key press events
        let key_controller = EventControllerKey::new();
        {
            let nes_clone = Arc::clone(&self.nes);
            key_controller.connect_key_pressed(
                glib::clone!(@weak window => @default-return glib::Propagation::Proceed, move |_, keyval, _, _| {
                    let mut nes = nes_clone.lock().unwrap();

                    // Map keys to NES controller buttons
                    match keyval {
                        // Player 1
                        Key::Left => nes.set_button_state(0, Button::Left, true),
                        Key::Right => nes.set_button_state(0, Button::Right, true),
                        Key::Up => nes.set_button_state(0, Button::Up, true),
                        Key::Down => nes.set_button_state(0, Button::Down, true),
                        Key::z => nes.set_button_state(0, Button::A, true),
                        Key::x => nes.set_button_state(0, Button::B, true),
                        Key::Return => nes.set_button_state(0, Button::Start, true),
                        Key::Shift_L | Key::Shift_R => nes.set_button_state(0, Button::Select, true),
                        Key::Escape => {
                            // Quit on Escape
                            if let Some(app) = window.application() {
                                app.quit();
                            }
                        }
                        _ => {}
                    }

                    glib::Propagation::Stop
                }),
            );
        }

        // Create a key controller for key release events
        let key_release_controller = EventControllerKey::new();
        {
            let nes_clone = Arc::clone(&self.nes);
            key_release_controller.connect_key_released(
                move |_, keyval, _, _| {
                    let mut nes = nes_clone.lock().unwrap();

                    // Map keys to NES controller buttons
                    match keyval {
                        // Player 1
                        Key::Left => nes.set_button_state(0, Button::Left, false),
                        Key::Right => nes.set_button_state(0, Button::Right, false),
                        Key::Up => nes.set_button_state(0, Button::Up, false),
                        Key::Down => nes.set_button_state(0, Button::Down, false),
                        Key::z => nes.set_button_state(0, Button::A, false),
                        Key::x => nes.set_button_state(0, Button::B, false),
                        Key::Return => nes.set_button_state(0, Button::Start, false),
                        Key::Shift_L | Key::Shift_R => nes.set_button_state(0, Button::Select, false),
                        _ => {}
                    }
Propagate(true)
                });
        }

        // Add the controllers to the drawing area
        drawing_area.add_controller(key_controller);
        drawing_area.add_controller(key_release_controller);

        // Make the drawing area focusable and request focus
        drawing_area.set_can_focus(true);
        drawing_area.grab_focus();

        // Add the drawing area to the window
        window.set_child(Some(&drawing_area));

        // Connect the window's close request
        window.connect_close_request(|window| {
            if let Some(app) = window.application() {
                app.quit();
            }
            glib::Propagation::Stop
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
