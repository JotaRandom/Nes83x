use std::cell::Cell;

bitflags! {
    /// PPU Control Register ($2000)
    /// Controls various PPU operations including:
    /// - Base nametable address
    /// - VRAM address increment per CPU read/write of PPUDATA
    /// - Sprite pattern table address for 8x8 sprites
    /// - Background pattern table address
    /// - Sprite size (8x8 or 8x16)
    /// - PPU master/slave mode (not used in NES)
    /// - Generate NMI at start of vertical blanking interval
    #[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
    pub struct ControlRegister: u8 {
        const NAMETABLE1              = 0b0000_0001;
        const NAMETABLE2              = 0b0000_0010;
        const VRAM_ADD_INCREMENT      = 0b0000_0100;
        const SPRITE_PATTERN_ADDR     = 0b0000_1000;
        const BACKGROUND_PATTERN_ADDR = 0b0001_0000;
        const SPRITE_SIZE             = 0b0010_0000;
        const PPU_MASTER_SLAVE        = 0b0100_0000;
        const GENERATE_NMI            = 0b1000_0000;
    }
}

bitflags! {
    /// PPU Mask Register ($2001)
    /// Controls rendering of sprites and backgrounds, and color effects
    #[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
    pub struct MaskRegister: u8 {
        const GRAYSCALE               = 0b0000_0001;
        const SHOW_BG_LEFTMOST        = 0b0000_0010;
        const SHOW_SPRITES_LEFTMOST   = 0b0000_0100;
        const SHOW_BACKGROUND         = 0b0000_1000;
        const SHOW_SPRITES            = 0b0001_0000;
        const EMPHASIZE_RED           = 0b0010_0000;
        const EMPHASIZE_GREEN         = 0b0100_0000;
        const EMPHASIZE_BLUE          = 0b1000_0000;
    }
}

bitflags! {
    /// PPU Status Register ($2002)
    /// Reflects the state of the PPU
    #[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
    pub struct StatusRegister: u8 {
        const SPRITE_OVERFLOW = 0b0010_0000;
        const SPRITE_ZERO_HIT = 0b0100_0000;
        const VBLANK_STARTED  = 0b1000_0000;
    }
}

/// Represents the PPU registers
#[derive(Debug)]
pub struct Registers {
    /// PPU Control Register ($2000)
    pub ctrl: ControlRegister,
    /// PPU Mask Register ($2001)
    pub mask: MaskRegister,
    /// PPU Status Register ($2002)
    pub status: Cell<StatusRegister>,
    /// OAM Address Register ($2003)
    pub oam_addr: u8,
    /// OAM Data Register ($2004)
    pub oam_data: u8,
    /// Scroll Register ($2005)
    pub scroll: u8,
    /// PPU Address Register ($2006)
    pub addr: u16,
    /// PPU Data Register ($2007)
    pub data: u8,
    /// OAM DMA Register ($4014)
    pub oam_dma: u8,
    /// Temporary VRAM address (15 bits)
    pub temp_addr: u16,
    /// Fine X scroll (3 bits)
    pub fine_x: u8,
    /// Write toggle for PPUADDR/PPUSCROLL
    pub write_toggle: bool,
}

impl Registers {
    /// Create a new set of PPU registers with default values
    pub fn new() -> Self {
        Registers {
            ctrl: ControlRegister::empty(),
            mask: MaskRegister::empty(),
            status: Cell::new(StatusRegister::empty()),
            oam_addr: 0,
            oam_data: 0,
            scroll: 0,
            addr: 0,
            data: 0,
            oam_dma: 0,
            temp_addr: 0,
            fine_x: 0,
            write_toggle: false,
        }
    }

    /// Reset all registers to their initial state
    pub fn reset(&mut self) {
        self.ctrl = ControlRegister::empty();
        self.mask = MaskRegister::empty();
        self.status.set(StatusRegister::empty());
        self.oam_addr = 0;
        self.oam_data = 0;
        self.scroll = 0;
        self.addr = 0;
        self.data = 0;
        self.oam_dma = 0;
        self.temp_addr = 0;
        self.fine_x = 0;
        self.write_toggle = false;
    }

    /// Get the base nametable address (0x2000, 0x2400, 0x2800, or 0x2C00)
    pub fn nametable_addr(&self) -> u16 {
        0x2000 | ((self.ctrl.bits() as u16 & 0b11) << 10)
    }

    /// Get the VRAM address increment per CPU read/write of PPUDATA
    pub fn vram_addr_increment(&self) -> u16 {
        if self.ctrl.contains(ControlRegister::VRAM_ADD_INCREMENT) {
            32
        } else {
            1
        }
    }

    /// Get the sprite pattern table address (0x0000 or 0x1000)
    pub fn sprite_pattern_addr(&self) -> u16 {
        if self.ctrl.contains(ControlRegister::SPRITE_PATTERN_ADDR) {
            0x1000
        } else {
            0x0000
        }
    }

    /// Get the background pattern table address (0x0000 or 0x1000)
    pub fn background_pattern_addr(&self) -> u16 {
        if self.ctrl.contains(ControlRegister::BACKGROUND_PATTERN_ADDR) {
            0x1000
        } else {
            0x0000
        }
    }

    /// Get the sprite size (8x8 or 8x16)
    pub fn sprite_size(&self) -> (u8, u8) {
        if self.ctrl.contains(ControlRegister::SPRITE_SIZE) {
            (8, 16) // 8x16 sprites
        } else {
            (8, 8) // 8x8 sprites
        }
    }

    /// Check if NMI is enabled on VBlank
    pub fn nmi_enabled(&self) -> bool {
        self.ctrl.contains(ControlRegister::GENERATE_NMI)
    }

    /// Get the status register
    pub fn status(&self) -> StatusRegister {
        self.status.get()
    }

    /// Set the status register
    pub fn set_status(&self, status: StatusRegister) {
        self.status.set(status);
    }
}
