//! iNES Mapper 4 (MMC3) implementation

use crate::nes::mapper::{Mapper, Mirroring};

/// Mapper 4: MMC3 (Memory Management Controller 3)
/// Also known as: MMC3, MMC6, Mapper 4, Mapper 118, Mapper 119
#[derive(Debug)]
pub struct MMC3 {
    prg_rom: Vec<u8>,
    chr_rom: Option<Vec<u8>>,
    prg_ram: [u8; 0x2000],

    // Mapper registers
    bank_select: u8,
    bank_data: u8,
    mirroring: Mirroring,
    prg_ram_enabled: bool,
    prg_ram_write_protect: bool,
    irq_enabled: bool,
    irq_reload: bool,
    irq_counter: u8,
    irq_latch: u8,
    irq_occurred: bool,

    // Bank registers
    r: [u8; 8],

    // Internal state
    prg_mode: bool, // false: 0x8000-0x9FFF swappable, true: 0xC000-0xDFFF swappable
    chr_mode: bool, // false: 2KB CHR banks, true: 1KB CHR banks
    a12_prev_state: bool,
    cycle_counter: u16,
}

impl MMC3 {
    /// Create a new MMC3 mapper
    pub fn new(prg_rom: Vec<u8>, chr_rom: Option<Vec<u8>>, mirroring: Mirroring) -> Self {
        MMC3 {
            prg_rom,
            chr_rom,
            prg_ram: [0; 0x2000],

            // Initialize registers
            bank_select: 0,
            bank_data: 0,
            mirroring,
            prg_ram_enabled: true,
            prg_ram_write_protect: false,
            irq_enabled: false,
            irq_reload: false,
            irq_counter: 0,
            irq_latch: 0,
            irq_occurred: false,

            // Initialize bank registers
            r: [0; 8],

            // Initialize internal state
            prg_mode: false,
            chr_mode: false,
            a12_prev_state: false,
            cycle_counter: 0,
        }
    }

    /// Get the current PRG bank at the specified index (0-3)
    fn get_prg_bank(&self, index: usize) -> usize {
        match index {
            0 => {
                if self.prg_mode {
                    self.r[6] as usize * 0x2000 / 0x2000
                } else {
                    (self.prg_rom.len() - 0x4000) / 0x2000
                }
            }
            1 => self.r[7] as usize * 0x2000 / 0x2000,
            2 => {
                if self.prg_mode {
                    (self.prg_rom.len() - 0x4000) / 0x2000
                } else {
                    self.r[6] as usize * 0x2000 / 0x2000
                }
            }
            3 => (self.prg_rom.len() - 0x2000) / 0x2000, // Fixed to last bank
            _ => 0,
        }
    }

    /// Get the current CHR bank at the specified index (0-7)
    fn get_chr_bank(&self, index: usize) -> usize {
        if self.chr_mode {
            // 1KB CHR banks
            match index {
                0 | 1 => (self.r[0] & 0xFE) as usize + (index & 0x01),
                2 | 3 => (self.r[1] & 0xFE) as usize + (index & 0x01),
                4 => self.r[2] as usize,
                5 => self.r[3] as usize,
                6 => self.r[4] as usize,
                7 => self.r[5] as usize,
                _ => 0,
            }
        } else {
            // 2KB CHR banks
            match index {
                0 => (self.r[0] >> 1) as usize * 2,
                1 => (self.r[0] >> 1) as usize * 2 + 1,
                2 => (self.r[1] >> 1) as usize * 2,
                3 => (self.r[1] >> 1) as usize * 2 + 1,
                4 => self.r[2] as usize,
                5 => self.r[3] as usize,
                6 => self.r[4] as usize,
                7 => self.r[5] as usize,
                _ => 0,
            }
        }
    }

    /// Handle A12 toggling for IRQ counter
    pub fn handle_a12_toggle(&mut self, a12_high: bool) {
        if a12_high && !self.a12_prev_state {
            // A12 rising edge
            self.clock_irq_counter();
        }
        self.a12_prev_state = a12_high;
    }

    /// Clock the IRQ counter
    fn clock_irq_counter(&mut self) {
        if self.irq_counter == 0 || self.irq_reload {
            self.irq_counter = self.irq_latch;
            self.irq_reload = false;
        } else {
            self.irq_counter = self.irq_counter.wrapping_sub(1);
        }

        if self.irq_counter == 0 && self.irq_enabled {
            self.irq_occurred = true;
        }
    }
}

impl Mapper for MMC3 {
    fn read(&self, addr: u16) -> u8 {
        match addr {
            // CHR-ROM (0x0000-0x1FFF)
            0x0000..=0x1FFF => {
                if let Some(chr) = &self.chr_rom {
                    let bank_size = if self.chr_mode {
                        0x0400usize
                    } else {
                        0x0800usize
                    }; // 1KB or 2KB
                    let bank_index = (addr / bank_size as u16) as usize;
                    let bank = self.get_chr_bank(bank_index);
                    let offset = (addr % bank_size as u16) as usize;
                    let chr_addr = bank * bank_size + offset;
                    if chr_addr < chr.len() {
                        chr[chr_addr]
                    } else {
                        0
                    }
                } else {
                    0 // CHR RAM not implemented
                }
            }

            // PRG-RAM (0x6000-0x7FFF)
            0x6000..=0x7FFF => {
                if self.prg_ram_enabled {
                    self.prg_ram[(addr - 0x6000) as usize]
                } else {
                    0xFF // Open bus behavior
                }
            }

            // PRG-ROM (0x8000-0xFFFF)
            0x8000..=0x9FFF => {
                let bank = self.get_prg_bank(if self.prg_mode { 2 } else { 0 });
                let offset = (addr - 0x8000) as usize;
                let prg_addr = bank * 0x2000 + offset;
                if prg_addr < self.prg_rom.len() {
                    self.prg_rom[prg_addr]
                } else {
                    0
                }
            }
            0xA000..=0xBFFF => {
                let bank = self.get_prg_bank(1);
                let offset = (addr - 0xA000) as usize;
                let prg_addr = bank * 0x2000 + offset;
                if prg_addr < self.prg_rom.len() {
                    self.prg_rom[prg_addr]
                } else {
                    0
                }
            }
            0xC000..=0xDFFF => {
                let bank = self.get_prg_bank(if self.prg_mode { 0 } else { 2 });
                let offset = (addr - 0xC000) as usize;
                let prg_addr = bank * 0x2000 + offset;
                if prg_addr < self.prg_rom.len() {
                    self.prg_rom[prg_addr]
                } else {
                    0
                }
            }
            0xE000..=0xFFFF => {
                let bank = self.get_prg_bank(3); // Fixed to last bank
                let offset = (addr - 0xE000) as usize;
                let prg_addr = bank * 0x2000 + offset;
                if prg_addr < self.prg_rom.len() {
                    self.prg_rom[prg_addr]
                } else {
                    0
                }
            }
            _ => 0xFF,
        }
    }

    fn write(&mut self, addr: u16, value: u8) {
        match addr {
            // CHR-RAM (0x0000-0x1FFF) - if no CHR-ROM
            0x0000..=0x1FFF => {
                if self.chr_rom.is_none() {
                    // CHR RAM not implemented yet
                }
            }

            // PRG-RAM (0x6000-0x7FFF)
            0x6000..=0x7FFF => {
                if self.prg_ram_enabled && !self.prg_ram_write_protect {
                    self.prg_ram[(addr - 0x6000) as usize] = value;
                }
            }

            // Bank select ($8000-$9FFE, even)
            0x8000..=0x9FFE if addr & 0x0001 == 0 => {
                self.bank_select = value & 0x07;
                self.prg_mode = (value & 0x40) != 0;
                self.chr_mode = (value & 0x80) != 0;
            }

            // Bank data ($8001-$9FFF, odd)
            0x8001..=0x9FFF if addr & 0x0001 == 1 => {
                self.bank_data = value;

                match self.bank_select {
                    // R0-R5: CHR banks
                    0..=5 => {
                        self.r[self.bank_select as usize] = value;
                    }
                    // R6: PRG ROM bank at $8000-$9FFF (or $C000-$DFFF)
                    6 => {
                        self.r[6] = value & 0x3F; // 6-bit bank number
                    }
                    // R7: PRG ROM bank at $A000-$BFFF
                    7 => {
                        self.r[7] = value & 0x3F; // 6-bit bank number
                    }
                    _ => {}
                }
            }

            // Mirroring ($A000-$BFFE, even)
            0xA000..=0xBFFE if addr & 0x0001 == 0 => {
                self.mirroring = if (value & 0x01) == 0 {
                    Mirroring::Vertical
                } else {
                    Mirroring::Horizontal
                };
            }

            // PRG-RAM protect ($A001-$BFFF, odd)
            0xA001..=0xBFFF if addr & 0x0001 == 1 => {
                self.prg_ram_enabled = (value & 0x80) != 0;
                self.prg_ram_write_protect = (value & 0x40) != 0;
            }

            // IRQ latch ($C000-$DFFE, even)
            0xC000..=0xDFFE if addr & 0x0001 == 0 => {
                self.irq_latch = value;
            }

            // IRQ reload ($C001-$DFFF, odd)
            0xC001..=0xDFFF if addr & 0x0001 == 1 => {
                self.irq_reload = true;
            }

            // IRQ disable ($E000-$FFFE, even)
            0xE000..=0xFFFE if addr & 0x0001 == 0 => {
                self.irq_enabled = false;
                self.irq_occurred = false;
            }

            // IRQ enable ($E001-$FFFF, odd)
            0xE001..=0xFFFF if addr & 0x0001 == 1 => {
                self.irq_enabled = true;
            }

            _ => {}
        }
    }

    fn prg_rom(&self) -> &[u8] {
        &self.prg_rom
    }

    fn prg_rom_mut(&mut self) -> &mut [u8] {
        &mut self.prg_rom
    }

    fn chr_rom(&self) -> Option<&[u8]> {
        self.chr_rom.as_deref()
    }

    fn chr_rom_mut(&mut self) -> Option<&mut [u8]> {
        self.chr_rom.as_deref_mut()
    }

    fn mirroring(&self) -> Mirroring {
        self.mirroring
    }

    fn reset(&mut self) {
        // Reset registers
        self.bank_select = 0;
        self.bank_data = 0;
        self.prg_ram_enabled = true;
        self.prg_ram_write_protect = false;
        self.irq_enabled = false;
        self.irq_reload = false;
        self.irq_counter = 0;
        self.irq_latch = 0;
        self.irq_occurred = false;

        // Reset bank registers
        self.r = [0; 8];

        // Reset internal state
        self.prg_mode = false;
        self.chr_mode = false;
        self.a12_prev_state = false;
        self.cycle_counter = 0;

        // Set initial bank values for PRG-ROM
        // Last bank - 1 at 0x8000-0x9FFF, Last bank at 0xA000-0xBFFF
        let num_prg_banks = self.prg_rom.len() / 0x2000;
        self.r[6] = (num_prg_banks - 2) as u8;
        self.r[7] = (num_prg_banks - 1) as u8;
    }

    // CHR memory access is handled through the chr_rom() and chr_rom_mut() methods
    // which are part of the Mapper trait
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_cartridge() -> (Vec<u8>, Option<Vec<u8>>, Mirroring) {
        // Create a test cartridge with 256KB PRG-ROM and 128KB CHR-ROM
        let prg_rom = vec![0; 256 * 1024];
        let chr_rom = Some(vec![0; 128 * 1024]);

        // Fill with test patterns
        for (i, byte) in prg_rom.iter_mut().enumerate() {
            *byte = (i & 0xFF) as u8;
        }

        if let Some(chr) = chr_rom.as_mut() {
            for (i, byte) in chr.iter_mut().enumerate() {
                *byte = (i & 0xFF) as u8;
            }
        }

        (prg_rom, chr_rom, Mirroring::Horizontal)
    }

    #[test]
    fn test_mmc3_initial_state() {
        let (prg_rom, chr_rom, mirroring) = create_test_cartridge();
        let mmc3 = MMC3::new(prg_rom, chr_rom, mirroring);

        assert_eq!(mmc3.bank_select, 0);
        assert!(!mmc3.prg_mode);
        assert!(!mmc3.chr_mode);
        assert!(!mmc3.irq_enabled);
        assert!(!mmc3.irq_occurred);
        // Initial bank values depend on ROM size
    }

    #[test]
    fn test_mmc3_prg_banking() {
        let (prg_rom, chr_rom, mirroring) = create_test_cartridge();
        let mut mmc3 = MMC3::new(prg_rom, chr_rom, mirroring);

        // Test bank switching
        mmc3.write(0x8000, 0x06); // Select PRG bank 0
        mmc3.write(0x8001, 0x12); // Set bank to 0x12

        // Verify bank switching
        assert_eq!(mmc3.r[6], 0x12);

        // Test PRG mode 0 (0x8000-0x9FFF swappable)
        mmc3.write(0x8000, 0x40); // Set PRG mode 0
        assert!(mmc3.prg_mode);

        // Test reading from PRG-ROM
        let bank = mmc3.get_prg_bank(0);
        assert_eq!(bank, 0x12);
    }

    #[test]
    fn test_mmc3_chr_banking() {
        let (prg_rom, chr_rom, mirroring) = create_test_cartridge();
        let mut mmc3 = MMC3::new(prg_rom, chr_rom, mirroring);

        // Test CHR bank 0 switching (2KB mode)
        mmc3.write(0x8000, 0x00); // Select CHR bank 0
        mmc3.write(0x8001, 0x12); // Set bank to 0x12

        // Verify CHR bank switching
        assert_eq!(mmc3.r[0], 0x12);

        // Test CHR mode 1 (1KB banks)
        mmc3.write(0x8000, 0x80); // Set CHR mode 1
        assert!(mmc3.chr_mode);
    }

    #[test]
    fn test_mmc3_irq() {
        let (prg_rom, chr_rom, mirroring) = create_test_cartridge();
        let mut mmc3 = MMC3::new(prg_rom, chr_rom, mirroring);

        // Set IRQ latch
        mmc3.write(0xC000, 0x04); // Set IRQ latch to 4
        assert_eq!(mmc3.irq_latch, 0x04);

        // Enable IRQ
        mmc3.write(0xE001, 0x00); // Enable IRQ
        assert!(mmc3.irq_enabled);

        // Trigger IRQ reload
        mmc3.write(0xC001, 0x00); // Reload IRQ counter

        // Simulate A12 toggles to trigger IRQ
        for _ in 0..4 {
            mmc3.handle_a12_toggle(true);
            mmc3.handle_a12_toggle(false);
        }

        // IRQ should be triggered
        assert!(mmc3.irq_occurred);

        // Disable IRQ
        mmc3.write(0xE000, 0x00); // Disable IRQ
        assert!(!mmc3.irq_enabled);
        assert!(!mmc3.irq_occurred);
    }
}
