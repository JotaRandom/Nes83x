//! Emulator frontend with GTK4 UI

// GTK4 imports
use gtk::gdk::Key;
use gtk::prelude::*;
use gtk::EventControllerKey;
use gtk::{Application, ApplicationWindow, DrawingArea};
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use thiserror::Error;
use log::{info, error};

use crate::nes::{input::Button as NesButton, Nes, NesError};

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
    drawing_area: Arc<Mutex<Option<DrawingArea>>>,

    /// The current ROM path, if any
    pub rom_path: Option<PathBuf>,

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
            drawing_area: Arc::new(Mutex::new(None)),
            rom_path: None,
            is_running: false,
        })
    }

    /// Initialize the emulator UI
    pub fn init(&mut self) -> Result<(), EmulatorError> {
        // Clone references for the closure
        let nes_clone = Arc::clone(&self.nes);
        let app_clone_for_activate = self.app.clone();
        let drawing_area_arc = Arc::clone(&self.drawing_area);

        // Connect the activate signal
        app_clone_for_activate.connect_activate(move |app| {
            eprintln!("GTK activate called");
            // Clone app for use in closures
            let app_clone = app.clone();
            let app_clone_for_key = app.clone();

            // Create the main window
            let window = ApplicationWindow::builder()
                .application(app)
                .title("Nes83x - NES Emulator")
                .default_width(256 * 2)  // 2x scale
                .default_height(240 * 2) // 2x scale
                .build();

            // Create the drawing area
            let drawing_area = DrawingArea::new();

            // Set size request
            drawing_area.set_size_request(256 * 2, 240 * 2);
            drawing_area.set_hexpand(true);
            drawing_area.set_vexpand(true);

            // Set up the drawing area
            let nes_clone_for_draw = Arc::clone(&nes_clone);
            drawing_area.set_draw_func(move |_, cr, width, height| {
                eprintln!("Draw called, width: {}, height: {}", width, height);
                
                // Get the NES framebuffer
                let nes = nes_clone_for_draw.lock().unwrap();
                let ppu = nes.ppu.borrow();
                let framebuffer = &ppu.framebuffer;
                
                // Scale to fit the drawing area
                let scale_x = width as f64 / 256.0;
                let scale_y = height as f64 / 240.0;
                
                // Draw the framebuffer pixel by pixel
                for y in 0..240 {
                    for x in 0..256 {
                        let idx = (y * 256 + x) * 4;
                        if idx + 3 < framebuffer.len() {
                            let a = framebuffer[idx] as f64 / 255.0;
                            let r = framebuffer[idx + 1] as f64 / 255.0;
                            let g = framebuffer[idx + 2] as f64 / 255.0;
                            let b = framebuffer[idx + 3] as f64 / 255.0;
                            
                            cr.set_source_rgba(r, g, b, a);
                            cr.rectangle(x as f64 * scale_x, y as f64 * scale_y, scale_x, scale_y);
                            let _ = cr.fill();
                        }
                    }
                }
                
                eprintln!("Framebuffer drawn");
            });

            // Store the drawing area
            *drawing_area_arc.lock().unwrap() = Some(drawing_area.clone());

            // Create a key controller for key press events
            let key_controller = EventControllerKey::new();
            {
                let nes_clone_inner = Arc::clone(&nes_clone);
                key_controller.connect_key_pressed(move |_, keyval, _, _| {
                    let mut nes = nes_clone_inner.lock().unwrap();

                    // Map keys to NES controller buttons
                    match keyval {
                        // Player 1
                        Key::Left => nes.set_button_state(0, NesButton::Left, true),
                        Key::Right => nes.set_button_state(0, NesButton::Right, true),
                        Key::Up => nes.set_button_state(0, NesButton::Up, true),
                        Key::Down => nes.set_button_state(0, NesButton::Down, true),
                        Key::z => nes.set_button_state(0, NesButton::A, true),
                        Key::x => nes.set_button_state(0, NesButton::B, true),
                        Key::Return => nes.set_button_state(0, NesButton::Start, true),
                        Key::Shift_L | Key::Shift_R => nes.set_button_state(0, NesButton::Select, true),
                        Key::Escape => {
                            // Quit on Escape
                            app_clone_for_key.quit();
                        }
                        _ => {}
                    }

                    gtk::glib::Propagation::Stop
                });
            }

            // Create a key controller for key release events
            let key_release_controller = EventControllerKey::new();
            {
                let nes_clone_inner = Arc::clone(&nes_clone);
                key_release_controller.connect_key_released(move |_, keyval, _, _| {
                    let mut nes = nes_clone_inner.lock().unwrap();

                    // Map keys to NES controller buttons
                    match keyval {
                        // Player 1
                        Key::Left => nes.set_button_state(0, NesButton::Left, false),
                        Key::Right => nes.set_button_state(0, NesButton::Right, false),
                        Key::Up => nes.set_button_state(0, NesButton::Up, false),
                        Key::Down => nes.set_button_state(0, NesButton::Down, false),
                        Key::z => nes.set_button_state(0, NesButton::A, false),
                        Key::x => nes.set_button_state(0, NesButton::B, false),
                        Key::Return => nes.set_button_state(0, NesButton::Start, false),
                        Key::Shift_L | Key::Shift_R => nes.set_button_state(0, NesButton::Select, false),
                        _ => {}
                    }
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

            // Show the window
            window.present();
            eprintln!("Window presented");

            // Force initial draw
            drawing_area.queue_draw();
        });

        Ok(())
    }

    /// Load a ROM file
    pub fn load_rom<P: Into<PathBuf>>(&mut self, path: P) -> Result<(), EmulatorError> {
        let path = path.into();

        info!("Loading ROM in GTK emulator: {}", path.display());

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

        // Start the emulation thread if not already running
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

        // Start the emulation loop at ~60 FPS
        if let Some(ref drawing_area) = *self.drawing_area.lock().unwrap() {
            let nes_clone = Arc::clone(&self.nes);
            let drawing_area_clone = drawing_area.clone();
            glib::timeout_add_local(16, move || {
                let mut nes = nes_clone.lock().unwrap();
                if nes.is_running {
                    eprintln!("Timeout triggered, nes.is_running: {}", nes.is_running);
                    let _ = nes.run_frame();
                    drawing_area_clone.queue_draw();
                    glib::Continue(true)
                } else {
                    glib::Continue(false)
                }
            });
        }

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
        eprintln!("About to run GTK app");
        self.app.run();
        eprintln!("GTK app finished");
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
