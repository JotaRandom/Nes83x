//! # NES Picture Processing Unit (PPU)
//!
//! This module implements the Ricoh RP2C02 PPU used in the NES.

mod registers;
mod memory;

pub use registers::*;
pub use memory::*;

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
    vram_addr: u16,     // Current VRAM address (15 bits)
    temp_vram_addr: u16, // Temporary VRAM address
    fine_x: u8,         // Fine X scroll (3 bits)
    write_toggle: bool,  // Toggle for PPUADDR/PPUSCROLL writes

    // OAM (Object Attribute Memory)
    oam_addr: u8,
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
}

impl Default for Ppu {
    fn default() -> Self {
        Self::new()
    }
}

impl Ppu {
    /// Create a new PPU instance
    pub fn new() -> Self {
        Ppu {
            regs: Registers::new(),
            memory: PpuMemory::new(),
            scanline: 0,
            cycle: 0,
            frame: 0,
            vram_addr: 0,
            temp_vram_addr: 0,
            fine_x: 0,
            write_toggle: false,
            oam_addr: 0,
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
        // TODO: Implement scanline rendering
        // This will handle background rendering, sprite evaluation, and pixel composition
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
                self.temp_vram_addr = (self.temp_vram_addr & 0xF3FF) |
                                    ((value as u16 & 0x03) << 10);
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
                    self.temp_vram_addr = (self.temp_vram_addr & 0x0C1F) |
                                         ((fine_y as u16) << 12) |
                                         ((y_scroll & 0x07) << 12) |
                                         ((y_scroll & 0x18) << 2) |
                                         ((y_scroll & 0xE0) << 2);
                }
                self.write_toggle = !self.write_toggle;
                Ok(())
            }

            // PPUADDR
            0x6 => {
                if !self.write_toggle {
                    // First write: high byte of address
                    self.temp_vram_addr = (self.temp_vram_addr & 0x00FF) |
                                         (((value & 0x3F) as u16) << 8);
                } else {
                    // Second write: low byte of address
                    self.temp_vram_addr = (self.temp_vram_addr & 0xFF00) |
                                         (value as u16);
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
                log::warn!("Unimplemented PPU register write: 0x{:04X} = 0x{:02X}", addr, value);
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
