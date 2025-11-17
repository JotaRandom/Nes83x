//! # NES Picture Processing Unit (PPU)
//!
//! This module implements the Ricoh RP2C02 PPU used in the NES.

mod memory;
mod registers;

pub use memory::*;
pub use registers::*;

use std::cell::Cell;

/// The NES PPU (Picture Processing Unit)
#[derive(Debug)]
pub struct Ppu {
    /// PPU registers
    pub regs: Registers,

    /// PPU memory (VRAM, palettes, etc.)
    pub memory: PpuMemory,

    // Internal state
    scanline: u16,
    cycle: u16,
    frame: u64,

    // Rendering state
    vram_addr: Cell<u16>,      // Current VRAM address (15 bits)
    temp_vram_addr: Cell<u16>, // Temporary VRAM address
    fine_x: Cell<u8>,          // Fine X scroll (3 bits)
    write_toggle: Cell<bool>,  // Toggle for PPUADDR/PPUSCROLL writes

    // OAM (Object Attribute Memory)
    oam_addr: Cell<u8>,
    oam_data: [u8; 256], // 64 sprites * 4 bytes each

    // Secondary OAM (used during sprite evaluation)
    secondary_oam: [u8; 32], // 8 sprites * 4 bytes

    // Rendering shift registers
    pattern_shift_lo: u16,
    pattern_shift_hi: u16,
    attr_shift_lo: u16,
    attr_shift_hi: u16,

    // Pixel rendering state
    bg_next_tile_id: u8,
    bg_next_tile_attr: u8,
    bg_next_tile_lo: u8,
    bg_next_tile_hi: u8,

    // NMI flag
    nmi_occurred: bool,
    nmi_previous: bool,
    nmi_delay: u8,

    // Framebuffer for rendering (256x240 ARGB pixels)
    pub framebuffer: Vec<u8>,
}

impl Default for Ppu {
    fn default() -> Self {
        Self::new()
    }
}

impl Ppu {
    /// Create a new PPU instance
    pub fn new() -> Self {
        let mut framebuffer = vec![0; 256 * 240 * 4];

        // Initialize with a test pattern (solid red)
        for pixel in framebuffer.chunks_mut(4) {
            pixel[0] = 0xFF; // B
            pixel[1] = 0x00; // G
            pixel[2] = 0xFF; // R
            pixel[3] = 0xFF; // A
        }

        Ppu {
            regs: Registers::new(),
            memory: PpuMemory::new(),
            scanline: 0,
            cycle: 0,
            frame: 0,
            vram_addr: Cell::new(0),
            temp_vram_addr: Cell::new(0),
            fine_x: Cell::new(0),
            write_toggle: Cell::new(false),
            oam_addr: Cell::new(0),
            oam_data: [0; 256],
            secondary_oam: [0; 32],
            pattern_shift_lo: 0,
            pattern_shift_hi: 0,
            attr_shift_lo: 0,
            attr_shift_hi: 0,
            bg_next_tile_id: 0,
            bg_next_tile_attr: 0,
            bg_next_tile_lo: 0,
            bg_next_tile_hi: 0,
            nmi_occurred: false,
            nmi_previous: false,
            nmi_delay: 0,
            framebuffer,
        }
    }

    /// Reset the PPU to its initial state
    pub fn reset(&mut self) {
        self.regs = Registers::new();
        self.memory = PpuMemory::new();
        self.scanline = 0;
        self.cycle = 0;
        self.frame = 0;
        self.vram_addr = 0;
        self.temp_vram_addr = 0;
        self.fine_x = 0;
        self.write_toggle = false;
        self.oam_addr = 0;
        // self.framebuffer = vec![0; 256 * 240 * 4]; // Keep the test pattern
        self.oam_data = [0; 256];
        self.secondary_oam = [0; 32];
        self.pattern_shift_lo = 0;
        self.pattern_shift_hi = 0;
        self.attr_shift_lo = 0;
        self.attr_shift_hi = 0;
        self.bg_next_tile_id = 0;
        self.bg_next_tile_attr = 0;
        self.bg_next_tile_lo = 0;
        self.bg_next_tile_hi = 0;
        self.nmi_occurred = false;
        self.nmi_previous = false;
        self.nmi_delay = 0;
    }

    /// Execute one PPU cycle
    pub fn tick(&mut self) -> bool {
        let mut nmi = false;

        // Handle NMI generation with proper timing
        if self.nmi_delay > 0 {
            self.nmi_delay -= 1;
            if self.nmi_delay == 0 && self.nmi_occurred && self.nmi_previous {
                nmi = true;
            }
        }

        // Handle rendering
        if self.scanline < 240 || self.scanline == 261 {
            // Visible scanlines or pre-render scanline
            self.tick_scanline();
        } else if self.scanline == 241 && self.cycle == 1 {
            // Start of vertical blanking
            self.regs.status.insert(StatusRegister::VBLANK_STARTED);

            // Trigger NMI if enabled
            if self.regs.ctrl.contains(ControlRegister::GENERATE_NMI) {
                self.nmi_occurred = true;
                self.nmi_delay = 15; // NMI is delayed by 15 PPU cycles
            }
        }

        // Increment cycle and scanline counters
        self.cycle += 1;
        if self.cycle > 340 {
            self.cycle = 0;
            self.scanline += 1;

            if self.scanline > 261 {
                self.scanline = 0;
                self.frame += 1;

                // Clear VBLANK flag at the start of the frame
                self.regs.status.remove(StatusRegister::VBLANK_STARTED);

                // Clear sprite 0 hit and overflow flags
                self.regs.status.remove(StatusRegister::SPRITE_ZERO_HIT);
                self.regs.status.remove(StatusRegister::SPRITE_OVERFLOW);
            }
        }

        nmi
    }

    /// Handle scanline rendering
    fn tick_scanline(&mut self) {
        // Basic tile-based rendering
        if self.scanline < 240 && self.cycle < 256 {
            let x = self.cycle as usize;
            let y = self.scanline as usize;

            // Calculate which tile this pixel belongs to
            let tile_x = x / 8;
            let tile_y = y / 8;
            let pixel_x = x % 8;
            let pixel_y = y % 8;

            // Get tile index from nametable (simplified - using nametable 0)
            let nametable_addr = 0x2000;
            let tile_index_addr = nametable_addr + (tile_y * 32 + tile_x) as u16;
            let tile_index = self.memory.read(tile_index_addr);

            // Get pattern data from CHR ROM
            let pattern_table = 0x0000; // Use pattern table 0
            let tile_addr = pattern_table + (tile_index as u16 * 16) + pixel_y as u16;

            // Read the two bit planes for this pixel row
            let plane0 = self.memory.read(tile_addr);
            let plane1 = self.memory.read(tile_addr + 8);

            // Extract the pixel color (2 bits per pixel)
            let bit0 = (plane0 >> (7 - pixel_x)) & 1;
            let bit1 = (plane1 >> (7 - pixel_x)) & 1;
            let color_index = (bit1 << 1) | bit0;

            // Get palette color (simplified - using background palette)
            let palette_addr = 0x3F00 + color_index as u16;
            let color_value = if color_index == 0 { 0x0F } else { 0x30 }; // Forzar colores para debug

            // Convert NES color to RGB (simplified palette)
            let (r, g, b) = nes_color_to_rgb(color_value);

            // Write to framebuffer
            let pixel_index = (y * 256 + x) * 4;
            if pixel_index + 3 < self.framebuffer.len() {
                self.framebuffer[pixel_index] = b;
                self.framebuffer[pixel_index + 1] = g;
                self.framebuffer[pixel_index + 2] = r;
                self.framebuffer[pixel_index + 3] = 255;
            }
        }
    }

    /// Read a byte from a PPU register
    pub fn read_register(&mut self, addr: u16) -> u8 {
        match addr % 8 {
            // PPUSTATUS
            0x2 => {
                let status = self.regs.status.bits();
                // Reading PPUSTATUS clears the write latch and the VBLANK flag
                self.regs.status.remove(StatusRegister::VBLANK_STARTED);
                self.write_toggle = false;
                status
            }

            // OAMDATA
            0x4 => self.oam_data[self.oam_addr as usize],

            // PPUDATA
            0x7 => {
                let addr = self.vram_addr;
                let value = self.memory.read(addr);

                // Buffered read - the value read is from the previous read address
                let result = if addr < 0x3F00 {
                    // Buffered read from VRAM
                    let buffered = self.memory.read(addr.wrapping_sub(0x1000));
                    buffered
                } else {
                    // Direct read from palette RAM
                    value
                };

                // Increment VRAM address
                self.increment_vram_addr();

                result
            }

            _ => 0, // Open bus behavior
        }
    }

    /// Write a byte to a PPU register
    pub fn write_register(&mut self, addr: u16, value: u8) -> std::io::Result<()> {
        match addr % 8 {
            // PPUCTRL
            0x0 => {
                let _old_ctrl = self.regs.ctrl.bits();
                self.regs.ctrl = ControlRegister::from_bits_truncate(value);

                // Update NMI flag if needed
                let nmi_enabled = self.regs.ctrl.contains(ControlRegister::GENERATE_NMI);
                if nmi_enabled && self.regs.status.contains(StatusRegister::VBLANK_STARTED) {
                    self.nmi_occurred = true;
                    self.nmi_delay = 15;
                }

                // Copy bits 0-1 of the control register to the temporary VRAM address
                self.temp_vram_addr =
                    (self.temp_vram_addr & 0xF3FF) | ((value as u16 & 0x03) << 10);
                Ok(())
            }

            // PPUMASK
            0x1 => {
                self.regs.mask = MaskRegister::from_bits_truncate(value);
                Ok(())
            }

            // OAMADDR
            0x3 => {
                self.oam_addr = value;
                Ok(())
            }

            // OAMDATA
            0x4 => {
                // Writes to OAMDATA during rendering (on the pre-render line and the visible lines 0-239)
                // are ignored
                if self.scanline >= 240 || self.cycle < 2 {
                    self.oam_data[self.oam_addr as usize] = value;
                }
                self.oam_addr = self.oam_addr.wrapping_add(1);
                Ok(())
            }

            // PPUSCROLL
            0x5 => {
                if !self.write_toggle {
                    // First write: X scroll
                    self.fine_x = value & 0x07;
                    let x_scroll = (value >> 3) as u16;
                    self.temp_vram_addr = (self.temp_vram_addr & 0x7FE0) | x_scroll;
                } else {
                    // Second write: Y scroll
                    let fine_y = value & 0x07;
                    let y_scroll = (value >> 3) as u16;
                    self.temp_vram_addr = (self.temp_vram_addr & 0x0C1F)
                        | ((fine_y as u16) << 12)
                        | ((y_scroll & 0x07) << 12)
                        | ((y_scroll & 0x18) << 2)
                        | ((y_scroll & 0xE0) << 2);
                }
                self.write_toggle = !self.write_toggle;
                Ok(())
            }

            // PPUADDR
            0x6 => {
                if !self.write_toggle {
                    // First write: high byte of address
                    self.temp_vram_addr =
                        (self.temp_vram_addr & 0x00FF) | (((value & 0x3F) as u16) << 8);
                } else {
                    // Second write: low byte of address
                    self.temp_vram_addr = (self.temp_vram_addr & 0xFF00) | (value as u16);
                    self.vram_addr = self.temp_vram_addr;
                }
                self.write_toggle = !self.write_toggle;
                Ok(())
            }

            // PPUDATA
            0x7 => {
                self.memory.write(self.vram_addr, value);
                self.increment_vram_addr();
                Ok(())
            }

            _ => {
                // Log unimplemented register writes for debugging
                log::warn!(
                    "Unimplemented PPU register write: 0x{:04X} = 0x{:02X}",
                    addr,
                    value
                );
                Ok(())
            }
        }
    }

    /// Increment the VRAM address based on the control register
    fn increment_vram_addr(&mut self) {
        if self.regs.ctrl.contains(ControlRegister::VRAM_ADD_INCREMENT) {
            self.vram_addr = self.vram_addr.wrapping_add(32);
        } else {
            self.vram_addr = self.vram_addr.wrapping_add(1);
        }
    }

    /// Check if an NMI should be generated
    pub fn poll_nmi(&mut self) -> bool {
        if self.nmi_delay > 0 {
            self.nmi_delay -= 1;
            if self.nmi_delay == 0 && self.nmi_occurred && self.nmi_previous {
                self.nmi_previous = false;
                return true;
            }
        }
        false
    }

    /// Get the current scanline (0-261)
    pub fn scanline(&self) -> u16 {
        self.scanline
    }

    /// Get the current cycle within the scanline (0-340)
    pub fn cycle(&self) -> u16 {
        self.cycle
    }

    /// Get the current frame number
    pub fn frame(&self) -> u64 {
        self.frame
    }
}

/// Convert NES color index to RGB values
fn nes_color_to_rgb(color_index: u8) -> (u8, u8, u8) {
    // NES color palette (simplified - these are the standard NES colors)
    const NES_PALETTE: [(u8, u8, u8); 64] = [
        (0x80, 0x80, 0x80), (0x00, 0x3D, 0xA6), (0x00, 0x12, 0xB0), (0x44, 0x00, 0x96),
        (0xA1, 0x00, 0x5E), (0xC7, 0x00, 0x28), (0xBA, 0x06, 0x00), (0x8C, 0x17, 0x00),
        (0x5C, 0x2F, 0x00), (0x10, 0x45, 0x00), (0x05, 0x4A, 0x00), (0x00, 0x47, 0x2E),
        (0x00, 0x41, 0x66), (0x00, 0x00, 0x00), (0x05, 0x05, 0x05), (0x05, 0x05, 0x05),
        (0xC7, 0xC7, 0xC7), (0x00, 0x77, 0xFF), (0x21, 0x55, 0xFF), (0x82, 0x37, 0xFA),
        (0xEB, 0x2F, 0xB5), (0xFF, 0x29, 0x50), (0xFF, 0x22, 0x00), (0xD6, 0x32, 0x00),
        (0xC4, 0x62, 0x00), (0x35, 0x80, 0x00), (0x05, 0x8F, 0x00), (0x00, 0x8A, 0x55),
        (0x00, 0x99, 0xCC), (0x21, 0x21, 0x21), (0x09, 0x09, 0x09), (0x09, 0x09, 0x09),
        (0xFF, 0xFF, 0xFF), (0x0F, 0xD7, 0xFF), (0x69, 0xA2, 0xFF), (0xD4, 0x80, 0xFF),
        (0xFF, 0x45, 0xF3), (0xFF, 0x61, 0x8B), (0xFF, 0x88, 0x33), (0xFF, 0x9C, 0x12),
        (0xFA, 0xBC, 0x20), (0x9F, 0xE3, 0x0E), (0x2B, 0xF0, 0x35), (0x0C, 0xF0, 0xA4),
        (0x05, 0xFB, 0xFF), (0x5E, 0x5E, 0x5E), (0x0D, 0x0D, 0x0D), (0x0D, 0x0D, 0x0D),
        (0xFF, 0xFF, 0xFF), (0xA6, 0xFC, 0xFF), (0xB3, 0xEC, 0xFF), (0xDA, 0xAB, 0xEB),
        (0xFF, 0xA8, 0xF9), (0xFF, 0xAB, 0xB3), (0xFF, 0xD2, 0xB0), (0xFF, 0xEF, 0xA6),
        (0xFF, 0xF7, 0x9C), (0xD7, 0xE8, 0x95), (0xA6, 0xED, 0xAF), (0xA2, 0xF2, 0xDA),
        (0x99, 0xFF, 0xFC), (0xDD, 0xDD, 0xDD), (0x11, 0x11, 0x11), (0x11, 0x11, 0x11),
    ];

    if (color_index as usize) < NES_PALETTE.len() {
        NES_PALETTE[color_index as usize]
    } else {
        (0, 0, 0) // Black for invalid colors
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ppu_reset() {
        let mut ppu = Ppu::new();
        ppu.reset();

        assert_eq!(ppu.scanline, 0);
        assert_eq!(ppu.cycle, 0);
        assert_eq!(ppu.frame, 0);
        assert_eq!(ppu.vram_addr, 0);
        assert_eq!(ppu.temp_vram_addr, 0);
        assert_eq!(ppu.fine_x, 0);
        assert!(!ppu.write_toggle);
        assert_eq!(ppu.oam_addr, 0);
    }

    #[test]
    fn test_ppu_register_read_write() {
        let mut ppu = Ppu::new();

        // Test PPUCTRL
        ppu.write_register(0x2000, 0x80); // Enable NMI
        assert!(ppu.regs.ctrl.contains(ControlRegister::GENERATE_NMI));

        // Test PPUMASK
        ppu.write_register(0x2001, 0x1E); // Show background and sprites
        assert_eq!(ppu.regs.mask.bits(), 0x1E);

        // Test PPUSTATUS
        ppu.regs.status.insert(StatusRegister::VBLANK_STARTED);
        let status = ppu.read_register(0x2002);
        assert!(status & 0x80 != 0); // VBLANK flag should be set
        assert!(!ppu.regs.status.contains(StatusRegister::VBLANK_STARTED)); // Should be cleared after read

        // Test OAMADDR and OAMDATA
        ppu.write_register(0x2003, 0x10); // Set OAM address
        ppu.write_register(0x2004, 0xAA); // Write to OAM
        ppu.write_register(0x2003, 0x10); // Set OAM address again
        assert_eq!(ppu.read_register(0x2004), 0xAA); // Should read back the same value

        // Test PPUSCROLL
        ppu.write_register(0x2005, 0x42); // First write (X scroll)
        ppu.write_register(0x2005, 0x84); // Second write (Y scroll)
        assert_eq!(ppu.fine_x, 0x02); // Fine X should be the lower 3 bits of X scroll

        // Test PPUADDR and PPUDATA
        ppu.write_register(0x2006, 0x20); // High byte
        ppu.write_register(0x2006, 0x00); // Low byte
        ppu.write_register(0x2007, 0x55); // Write to VRAM

        // Read back (should be buffered)
        ppu.write_register(0x2006, 0x20);
        ppu.write_register(0x2006, 0x00);
        ppu.read_register(0x2007); // Dummy read
        assert_eq!(ppu.read_register(0x2007), 0x55);
    }
}
