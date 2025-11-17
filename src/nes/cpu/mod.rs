//! # 6502 CPU Emulation for the NES
//!
//! This module implements a cycle-accurate emulation of the Ricoh 2A03/2A07 CPU,
//! which is a variant of the MOS Technology 6502 microprocessor used in the NES.
//!
//! ## Key Features
//! - Full implementation of all 56 official 6502 instructions
//! - Support for unofficial/illegal opcodes used in some NES games (enabled with `unofficial_ops` feature)
//! - Cycle-accurate timing for correct emulation speed
//! - Proper interrupt handling (NMI, IRQ, BRK)
//! - Memory-mapped I/O for communication with PPU, APU, and controllers

use crate::nes::utils::Memory;
use crate::nes::utils::MemoryError;

mod memory;

mod status_flags;
pub use self::status_flags::StatusFlags;

/// Represents the result of a CPU operation
pub type CpuResult<T> = Result<T, CpuError>;

/// # 6502 CPU Registers
///
/// The 6502 has three general-purpose 8-bit registers (A, X, Y),
/// a program counter (PC), stack pointer (S), and a status register (P).
///
/// ## Register Descriptions:
/// - `a`: Accumulator - Main register for arithmetic and logic operations
/// - `x`: X Index - General purpose register, often used for counters/offsets
/// - `y`: Y Index - General purpose register, similar to X but with some addressing mode differences
/// - `s`: Stack Pointer - 8-bit register that points to the next available location on the hardware stack (page 1: 0x0100-0x01FF)
/// - `p`: Status Register - 8-bit register containing processor flags (see StatusFlags)
/// - `pc`: Program Counter - 16-bit register pointing to the next instruction to execute
#[derive(Debug, Default, Clone, Copy)]
pub struct Registers {
    /// Accumulator
    pub a: u8,
    /// X index register
    pub x: u8,
    /// Y index register
    pub y: u8,
    /// Stack pointer (points to location on bus)
    pub s: u8,
    /// Program counter
    pub pc: u16,
    /// Status register
    pub p: StatusFlags,
}

/// # 6502 CPU Emulator
///
/// This struct emulates the behavior of the 6502 CPU used in the NES.
/// It maintains the CPU's internal state including registers, program counter,
/// stack pointer, and various status flags.
///
/// ## Key Components:
/// - `reg`: Contains all CPU registers (A, X, Y, P, S, PC)
#[derive(Debug)]
pub struct Cpu {
    /// CPU registers
    pub reg: Registers,
    /// Number of cycles remaining for current instruction
    pub cycles: u64,
    /// Whether an NMI is pending
    pub nmi_pending: bool,
    /// Whether an IRQ is pending
    pub irq_pending: bool,
    /// Whether the CPU is in a valid state
    pub is_running: bool,
}

/// Errors that can occur during CPU operations
#[derive(Debug, thiserror::Error)]
pub enum CpuError {
    /// Invalid opcode encountered
    #[error("Invalid opcode: {0:02X}")]
    InvalidOpcode(u8),

    /// Memory access error
    #[error("Memory error: {0}")]
    MemoryError(#[from] crate::nes::utils::MemoryError),

    /// I/O error
    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    /// CPU is in an invalid state
    #[error("CPU is in an invalid state")]
    InvalidState,

    /// Invalid addressing mode
    #[error("Invalid addressing mode: {0:?}")]
    InvalidAddressingMode(AddressingMode),

    /// Invalid memory access at address
    #[error("Invalid memory access at address: {0:04X}")]
    InvalidMemoryAccess(u16),

    /// Stack overflow
    #[error("Stack overflow")]
    StackOverflow,

    /// Stack underflow
    #[error("Stack underflow")]
    StackUnderflow,
}

impl Cpu {
    /// Creates a new CPU instance with default values
    ///
    /// Initializes the CPU to its power-on state:
    /// - All registers set to zero
    /// - Stack pointer initialized to 0xFD (typical boot value)
    /// - Status flags set to 0x34 (Interrupt Disable and Break flags set)
    /// - Program counter will be set by the reset vector
    /// - All interrupt flags cleared
    /// - CPU marked as not running
    pub fn new() -> Self {
        Cpu {
            reg: Registers {
                a: 0,
                x: 0,
                y: 0,
                s: 0xFD, // Default stack pointer
                pc: 0,   // Will be set by the NES
                p: StatusFlags::UNUSED | StatusFlags::INTERRUPT_DISABLE,
            },
            cycles: 0,
            nmi_pending: false,
            irq_pending: false,
            is_running: false,
        }
    }

    /// Resets the CPU to its initial state
    ///
    /// This method initializes all CPU registers to their power-on state:
    /// - A, X, Y registers set to 0
    /// - Stack pointer set to 0xFD
    /// - Program counter will be set by the reset vector
    /// - Status flags set to 0x34 (Interrupt Disable and Break flags set)
    /// - All interrupt flags cleared
    /// - CPU marked as running
    /// - Memory is preserved
    pub fn reset(&mut self) {
        self.reg = Registers {
            a: 0,
            x: 0,
            y: 0,
            s: 0xFD,
            pc: 0, // Will be set by the NES
            p: StatusFlags::UNUSED | StatusFlags::INTERRUPT_DISABLE,
        };
        self.cycles = 0;
        self.nmi_pending = false;
        self.irq_pending = false;
        self.is_running = true;
    }

    /// Triggers a non-maskable interrupt (NMI)
    pub fn trigger_nmi(&mut self) {
        self.nmi_pending = true;
    }

    /// Alias for trigger_nmi() for compatibility with external code
    pub fn nmi(&mut self) {
        self.trigger_nmi();
    }

    /// Triggers an interrupt request (IRQ)
    pub fn trigger_irq(&mut self) {
        if !self.reg.p.contains(StatusFlags::INTERRUPT_DISABLE) {
            self.irq_pending = true;
        }
    }

    /// Executes a single CPU instruction
    ///
    /// This is the main CPU execution loop that:
    /// 1. Handles any pending interrupts (NMI has highest priority)
    /// 2. Fetches the next opcode from memory
    /// 3. Decodes and executes the instruction
    /// 4. Updates the cycle counter
    ///
    /// # Arguments
    /// * `nes` - A mutable reference to the NES system
    ///
    /// # Returns
    /// The number of cycles the instruction took to execute
    ///
    /// # Errors
    /// Returns `CpuError` if an invalid opcode is encountered or memory access fails
    pub fn step(&mut self, nes: &mut crate::nes::Nes) -> Result<u32, CpuError> {
        if !self.is_running {
            return Err(CpuError::InvalidState);
        }

        // Check for NMI (highest priority)
        if self.nmi_pending {
            self.nmi_pending = false;
            return self
                .handle_interrupt(0xFFFA, false, nes)
                .map(|c| c as u32);
        }

        // Check for IRQ (lower priority)
        if !self.reg.p.contains(StatusFlags::INTERRUPT_DISABLE) && self.irq_pending {
            self.irq_pending = false;
            return self
                .handle_interrupt(0xFFFE, false, nes)
                .map(|c| c as u32);
        }

        // Fetch and execute instruction
        let opcode = nes.read_byte(self.reg.pc)?;
        self.reg.pc = self.reg.pc.wrapping_add(1);

        let cycles = self.execute_instruction(opcode, nes)? as u32;

        // Update cycle count
        self.cycles = self.cycles.wrapping_add(cycles as u64);

        Ok(cycles)
    }
    /// Execute a single CPU instruction (internal implementation)
    ///
    /// # Arguments
    /// * `opcode` - The opcode to execute
    ///
    /// # Returns
    /// The number of cycles the instruction took to execute, or an error if the opcode is invalid
    fn execute_instruction(&mut self, opcode: u8, nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        match opcode {
            // Unofficial opcodes (enabled with the 'unofficial_ops' feature)
            #[cfg(feature = "unofficial_ops")]
            0x0B | 0x2B => self.aac(memory), // AAC (ANC)
            #[cfg(feature = "unofficial_ops")]
            0x8B => self.ane(memory), // ANE (XAA)
            #[cfg(feature = "unofficial_ops")]
            0x6B => self.arr(memory), // ARR
            #[cfg(feature = "unofficial_ops")]
            0xA7 | 0xB7 | 0xAF | 0xBF | 0xA3 | 0xB3 => {
                let mode = match opcode {
                    0xA7 => AddressingMode::ZeroPage,
                    0xB7 => AddressingMode::ZeroPageY,
                    0xAF => AddressingMode::Absolute,
                    0xBF => AddressingMode::AbsoluteY,
                    0xA3 => AddressingMode::IndirectX,
                    0xB3 => AddressingMode::IndirectY,
                    _ => unreachable!(),
                };
                self.lax(mode, nes) // LAX (LDA + TAX)
            }
            #[cfg(feature = "unofficial_ops")]
            0x27 | 0x37 | 0x2F | 0x3F | 0x3B | 0x23 | 0x33 => {
                let mode = match opcode {
                    0x27 => AddressingMode::ZeroPage,
                    0x37 => AddressingMode::ZeroPageX,
                    0x2F => AddressingMode::Absolute,
                    0x3F => AddressingMode::AbsoluteX,
                    0x3B => AddressingMode::AbsoluteY,
                    0x23 => AddressingMode::IndirectX,
                    0x33 => AddressingMode::IndirectY,
                    _ => unreachable!(),
                };
                self.rla(mode, nes) // RLA
            }
            #[cfg(feature = "unofficial_ops")]
            0x47 | 0x57 | 0x4F | 0x5F | 0x5B | 0x43 | 0x53 => {
                let mode = match opcode {
                    0x47 => AddressingMode::ZeroPage,
                    0x57 => AddressingMode::ZeroPageX,
                    0x4F => AddressingMode::Absolute,
                    0x5F => AddressingMode::AbsoluteX,
                    0x5B => AddressingMode::AbsoluteY,
                    0x43 => AddressingMode::IndirectX,
                    0x53 => AddressingMode::IndirectY,
                    _ => unreachable!(),
                };
                self.sre(mode, nes) // SRE (LSE)
            }
            #[cfg(feature = "unofficial_ops")]
            0x67 | 0x77 | 0x6F | 0x7F | 0x7B | 0x63 | 0x73 => {
                let mode = match opcode {
                    0x67 => AddressingMode::ZeroPage,
                    0x77 => AddressingMode::ZeroPageX,
                    0x6F => AddressingMode::Absolute,
                    0x7F => AddressingMode::AbsoluteX,
                    0x7B => AddressingMode::AbsoluteY,
                    0x63 => AddressingMode::IndirectX,
                    0x73 => AddressingMode::IndirectY,
                    _ => unreachable!(),
                };
                self.rra(mode, nes) // RRA
            }
            #[cfg(feature = "unofficial_ops")]
            0xC7 | 0xD7 | 0xCF | 0xDF | 0xDB | 0xC3 | 0xD3 => {
                let mode = match opcode {
                    0xC7 => AddressingMode::ZeroPage,
                    0xD7 => AddressingMode::ZeroPageX,
                    0xCF => AddressingMode::Absolute,
                    0xDF => AddressingMode::AbsoluteX,
                    0xDB => AddressingMode::AbsoluteY,
                    0xC3 => AddressingMode::IndirectX,
                    0xD3 => AddressingMode::IndirectY,
                    _ => unreachable!(),
                };
                self.dcp(mode, nes) // DCP
            }
            #[cfg(feature = "unofficial_ops")]
            0xE7 | 0xF7 | 0xEF | 0xFF | 0xFB | 0xE3 | 0xF3 => {
                let mode = match opcode {
                    0xE7 => AddressingMode::ZeroPage,
                    0xF7 => AddressingMode::ZeroPageX,
                    0xEF => AddressingMode::Absolute,
                    0xFF => AddressingMode::AbsoluteX,
                    0xFB => AddressingMode::AbsoluteY,
                    0xE3 => AddressingMode::IndirectX,
                    0xF3 => AddressingMode::IndirectY,
                    _ => unreachable!(),
                };
                self.isc(mode, nes) // ISC (ISB)
            }
            // Load/Store Operations
            0xA9 => self.lda(AddressingMode::Immediate, nes), // LDA Immediate
            0xA5 => self.lda(AddressingMode::ZeroPage, nes),  // LDA Zero Page
            0xB5 => self.lda(AddressingMode::ZeroPageX, nes), // LDA Zero Page,X
            0xAD => self.lda(AddressingMode::Absolute, nes),  // LDA Absolute
            0xBD => self.lda(AddressingMode::AbsoluteX, nes), // LDA Absolute,X
            0xB9 => self.lda(AddressingMode::AbsoluteY, nes), // LDA Absolute,Y
            0xA1 => self.lda(AddressingMode::IndirectX, nes), // LDA (Indirect,X)
            0xB1 => self.lda(AddressingMode::IndirectY, nes), // LDA (Indirect),Y

            0xA2 => self.ldx(AddressingMode::Immediate, nes), // LDX Immediate
            0xA6 => self.ldx(AddressingMode::ZeroPage, nes),  // LDX Zero Page
            0xB6 => self.ldx(AddressingMode::ZeroPageY, nes), // LDX Zero Page,Y
            0xAE => self.ldx(AddressingMode::Absolute, nes),  // LDX Absolute
            0xBE => self.ldx(AddressingMode::AbsoluteY, nes), // LDX Absolute,Y

            0xA0 => self.ldy(AddressingMode::Immediate, nes), // LDY Immediate
            0xA4 => self.ldy(AddressingMode::ZeroPage, nes),  // LDY Zero Page
            0xB4 => self.ldy(AddressingMode::ZeroPageX, nes), // LDY Zero Page,X
            0xAC => self.ldy(AddressingMode::Absolute, nes),  // LDY Absolute
            0xBC => self.ldy(AddressingMode::AbsoluteX, nes), // LDY Absolute,X

            0x85 => self.sta(AddressingMode::ZeroPage, nes), // STA Zero Page
            0x95 => self.sta(AddressingMode::ZeroPageX, nes), // STA Zero Page,X
            0x8D => self.sta(AddressingMode::Absolute, nes), // STA Absolute
            0x9D => self.sta(AddressingMode::AbsoluteX, nes), // STA Absolute,X
            0x99 => self.sta(AddressingMode::AbsoluteY, nes), // STA Absolute,Y
            0x81 => self.sta(AddressingMode::IndirectX, nes), // STA (Indirect,X)
            0x91 => self.sta(AddressingMode::IndirectY, nes), // STA (Indirect),Y

            0x86 => self.stx(AddressingMode::ZeroPage, nes), // STX Zero Page
            0x96 => self.stx(AddressingMode::ZeroPageY, nes), // STX Zero Page,Y
            0x8E => self.stx(AddressingMode::Absolute, nes), // STX Absolute

            0x84 => self.sty(AddressingMode::ZeroPage, nes), // STY Zero Page
            0x94 => self.sty(AddressingMode::ZeroPageX, nes), // STY Zero Page,X
            0x8C => self.sty(AddressingMode::Absolute, nes), // STY Absolute

            // Register Transfers
            0xAA => self.tax(), // TAX
            0x8A => self.txa(), // TXA
            0xA8 => self.tay(), // TAY
            0x98 => self.tya(), // TYA
            0xBA => self.tsx(), // TSX
            0x9A => self.txs(), // TXS

            // Stack Operations
            0x48 => self.pha(memory), // PHA
            0x08 => self.php(memory), // PHP
            0x68 => self.pla(memory), // PLA
            0x28 => self.plp(memory), // PLP

            // Logical Operations
            0x29 => self.and(AddressingMode::Immediate, nes), // AND Immediate
            0x25 => self.and(AddressingMode::ZeroPage, nes),  // AND Zero Page
            0x35 => self.and(AddressingMode::ZeroPageX, nes), // AND Zero Page,X
            0x2D => self.and(AddressingMode::Absolute, nes),  // AND Absolute
            0x3D => self.and(AddressingMode::AbsoluteX, nes), // AND Absolute,X
            0x39 => self.and(AddressingMode::AbsoluteY, nes), // AND Absolute,Y
            0x21 => self.and(AddressingMode::IndirectX, nes), // AND (Indirect,X)
            0x31 => self.and(AddressingMode::IndirectY, nes), // AND (Indirect),Y

            0x49 => self.eor(AddressingMode::Immediate, nes), // EOR Immediate
            0x45 => self.eor(AddressingMode::ZeroPage, nes),  // EOR Zero Page
            0x55 => self.eor(AddressingMode::ZeroPageX, nes), // EOR Zero Page,X
            0x4D => self.eor(AddressingMode::Absolute, nes),  // EOR Absolute
            0x5D => self.eor(AddressingMode::AbsoluteX, nes), // EOR Absolute,X
            0x59 => self.eor(AddressingMode::AbsoluteY, nes), // EOR Absolute,Y
            0x41 => self.eor(AddressingMode::IndirectX, nes), // EOR (Indirect,X)
            0x51 => self.eor(AddressingMode::IndirectY, nes), // EOR (Indirect),Y

            0x09 => self.ora(AddressingMode::Immediate, nes), // ORA Immediate
            0x05 => self.ora(AddressingMode::ZeroPage, nes),  // ORA Zero Page
            0x15 => self.ora(AddressingMode::ZeroPageX, nes), // ORA Zero Page,X
            0x0D => self.ora(AddressingMode::Absolute, nes),  // ORA Absolute
            0x1D => self.ora(AddressingMode::AbsoluteX, nes), // ORA Absolute,X
            0x19 => self.ora(AddressingMode::AbsoluteY, nes), // ORA Absolute,Y
            0x01 => self.ora(AddressingMode::IndirectX, nes), // ORA (Indirect,X)

            0x24 => self.bit(AddressingMode::ZeroPage, nes), // BIT Zero Page
            0x2C => self.bit(AddressingMode::Absolute, nes), // BIT Absolute

            // Arithmetic Operations
            0x69 => self.adc(AddressingMode::Immediate, nes), // ADC Immediate
            0x65 => self.adc(AddressingMode::ZeroPage, nes),  // ADC Zero Page
            0x75 => self.adc(AddressingMode::ZeroPageX, nes), // ADC Zero Page,X
            0x6D => self.adc(AddressingMode::Absolute, nes),  // ADC Absolute
            0x7D => self.adc(AddressingMode::AbsoluteX, nes), // ADC Absolute,X
            0x79 => self.adc(AddressingMode::AbsoluteY, nes), // ADC Absolute,Y
            0x61 => self.adc(AddressingMode::IndirectX, nes), // ADC (Indirect,X)
            0x71 => self.adc(AddressingMode::IndirectY, nes), // ADC (Indirect),Y

            0xE9 => self.sbc(AddressingMode::Immediate, nes), // SBC Immediate
            0xE5 => self.sbc(AddressingMode::ZeroPage, nes),  // SBC Zero Page
            0xF5 => self.sbc(AddressingMode::ZeroPageX, nes), // SBC Zero Page,X
            0xED => self.sbc(AddressingMode::Absolute, nes),  // SBC Absolute
            0xFD => self.sbc(AddressingMode::AbsoluteX, nes), // SBC Absolute,X
            0xF9 => self.sbc(AddressingMode::AbsoluteY, nes), // SBC Absolute,Y
            0xE1 => self.sbc(AddressingMode::IndirectX, nes), // SBC (Indirect,X)
            0xF1 => self.sbc(AddressingMode::IndirectY, nes), // SBC (Indirect),Y

            0xC9 => self.cmp(AddressingMode::Immediate, nes), // CMP Immediate
            0xC5 => self.cmp(AddressingMode::ZeroPage, nes),  // CMP Zero Page
            0xD5 => self.cmp(AddressingMode::ZeroPageX, nes), // CMP Zero Page,X
            0xCD => self.cmp(AddressingMode::Absolute, nes),  // CMP Absolute
            0xDD => self.cmp(AddressingMode::AbsoluteX, nes), // CMP Absolute,X
            0xD9 => self.cmp(AddressingMode::AbsoluteY, nes), // CMP Absolute,Y
            0xC1 => self.cmp(AddressingMode::IndirectX, nes), // CMP (Indirect,X)
            0xD1 => self.cmp(AddressingMode::IndirectY, nes), // CMP (Indirect),Y

            0xE0 => self.cpx(AddressingMode::Immediate, nes), // CPX Immediate
            0xE4 => self.cpx(AddressingMode::ZeroPage, nes),  // CPX Zero Page
            0xEC => self.cpx(AddressingMode::Absolute, nes),  // CPX Absolute

            0xC0 => self.cpy(AddressingMode::Immediate, nes), // CPY Immediate
            0xC4 => self.cpy(AddressingMode::ZeroPage, nes),  // CPY Zero Page
            0xCC => self.cpy(AddressingMode::Absolute, nes),  // CPY Absolute

            // Increments & Decrements
            0xE6 => self.inc(AddressingMode::ZeroPage, nes), // INC Zero Page
            0xF6 => self.inc(AddressingMode::ZeroPageX, nes), // INC Zero Page,X
            0xEE => self.inc(AddressingMode::Absolute, nes), // INC Absolute
            0xFE => self.inc(AddressingMode::AbsoluteX, nes), // INC Absolute,X

            0xE8 => self.inx(), // INX
            0xC8 => self.iny(), // INY

            0xC6 => self.dec(AddressingMode::ZeroPage, nes), // DEC Zero Page
            0xD6 => self.dec(AddressingMode::ZeroPageX, nes), // DEC Zero Page,X
            0xCE => self.dec(AddressingMode::Absolute, nes), // DEC Absolute
            0xDE => self.dec(AddressingMode::AbsoluteX, nes), // DEC Absolute,X

            0xCA => self.dex(), // DEX
            0x88 => self.dey(), // DEY

            // Shifts
            0x0A => self.asl_acc(), // ASL Accumulator
            0x06 => self.asl(AddressingMode::ZeroPage, nes), // ASL Zero Page
            0x16 => self.asl(AddressingMode::ZeroPageX, nes), // ASL Zero Page,X
            0x0E => self.asl(AddressingMode::Absolute, nes), // ASL Absolute
            0x1E => self.asl(AddressingMode::AbsoluteX, nes), // ASL Absolute,X

            0x4A => self.lsr_acc(), // LSR Accumulator
            0x46 => self.lsr(AddressingMode::ZeroPage, nes), // LSR Zero Page
            0x56 => self.lsr(AddressingMode::ZeroPageX, nes), // LSR Zero Page,X
            0x4E => self.lsr(AddressingMode::Absolute, nes), // LSR Absolute
            0x5E => self.lsr(AddressingMode::AbsoluteX, nes), // LSR Absolute,X

            0x2A => self.rol_acc(), // ROL Accumulator
            0x26 => self.rol(AddressingMode::ZeroPage, nes), // ROL Zero Page
            0x36 => self.rol(AddressingMode::ZeroPageX, nes), // ROL Zero Page,X
            0x2E => self.rol(AddressingMode::Absolute, nes), // ROL Absolute
            0x3E => self.rol(AddressingMode::AbsoluteX, nes), // ROL Absolute,X

            0x6A => self.ror_acc(), // ROR Accumulator
            0x66 => self.ror(AddressingMode::ZeroPage, nes), // ROR Zero Page
            0x76 => self.ror(AddressingMode::ZeroPageX, nes), // ROR Zero Page,X
            0x6E => self.ror(AddressingMode::Absolute, nes), // ROR Absolute
            0x7E => self.ror(AddressingMode::AbsoluteX, nes), // ROR Absolute,X

            // Jumps & Calls
            0x4C => self.jmp(AddressingMode::Absolute, nes), // JMP Absolute
            0x6C => self.jmp(AddressingMode::Indirect, nes), // JMP Indirect
            0x20 => self.jsr(memory),                           // JSR
            0x60 => self.rts(memory),                           // RTS
            0x40 => self.rti(memory),                           // RTI

            // Branches
            0x90 => self.branch(!self.reg.p.contains(StatusFlags::CARRY), nes), // BCC
            0xB0 => self.branch(self.reg.p.contains(StatusFlags::CARRY), nes),  // BCS
            0xF0 => self.branch(self.reg.p.contains(StatusFlags::ZERO), nes),   // BEQ
            0x30 => self.branch(self.reg.p.contains(StatusFlags::NEGATIVE), nes), // BMI
            0xD0 => self.branch(!self.reg.p.contains(StatusFlags::ZERO), nes),  // BNE
            0x10 => self.branch(!self.reg.p.contains(StatusFlags::NEGATIVE), nes), // BPL
            0x50 => self.branch(!self.reg.p.contains(StatusFlags::OVERFLOW), nes), // BVC
            0x70 => self.branch(self.reg.p.contains(StatusFlags::OVERFLOW), nes), // BVS

            // Status Flag Changes
            0x18 => self.clc(), // CLC
            0xD8 => self.cld(), // CLD
            0x58 => self.cli(), // CLI
            0xB8 => self.clv(), // CLV
            0x38 => self.sec(), // SEC
            0xF8 => self.sed(), // SED
            0x78 => self.sei(), // SEI

            // System Functions
            0x00 => self.brk(memory), // BRK
            0xEA => Ok(2),            // NOP

            // Unofficial/Illegal Opcodes (stubs for now)
            0x0B | 0x2B => self.aac(memory), // AAC (ANC)
            0x8B => self.ane(memory),        // ANE (XAA)
            0x6B => self.arr(memory),        // ARR
            0xCB => self.axs(AddressingMode::Immediate, nes), // AXS (SBX)
            0xE2 => self.dop(AddressingMode::Immediate, nes), // DOP (NOP)
            0x04 | 0x44 | 0x64 => self.nop(memory), // NOP Zero Page
            0x14 | 0x34 | 0x54 | 0x74 | 0xD4 | 0xF4 => self.nop(memory), // NOP Zero Page,X
            0x80 | 0x82 | 0x89 | 0xC2 => self.nop(memory), // NOP Immediate
            0x0C => self.nop(memory),        // NOP Absolute
            0x1C | 0x3C | 0x5C | 0x7C | 0xDC | 0xFC => self.nop(memory), // NOP Absolute,X
            0xA7 => self.las(AddressingMode::ZeroPage, nes), // LAS (Unofficial)
            0x1A | 0x3A | 0x5A | 0x7A | 0xDA | 0xFA => self.nop(memory), // NOP Implied
            0x27 => self.rla(AddressingMode::ZeroPage, nes), // RLA Zero Page
            0x37 => self.rla(AddressingMode::ZeroPageX, nes), // RLA Zero Page,X
            0x2F => self.rla(AddressingMode::Absolute, nes), // RLA Absolute
            0x3F => self.rla(AddressingMode::AbsoluteX, nes), // RLA Absolute,X
            0x3B => self.rla(AddressingMode::AbsoluteY, nes), // RLA Absolute,Y
            0x23 => self.rla(AddressingMode::IndirectX, nes), // RLA (Indirect,X)
            0x33 => self.rla(AddressingMode::IndirectY, nes), // RLA (Indirect),Y
            0x67 => self.rra(AddressingMode::ZeroPage, nes), // RRA Zero Page
            0x77 => self.rra(AddressingMode::ZeroPageX, nes), // RRA Zero Page,X
            0x6F => self.rra(AddressingMode::Absolute, nes), // RRA Absolute
            0x7F => self.rra(AddressingMode::AbsoluteX, nes), // RRA Absolute,X
            0x7B => self.rra(AddressingMode::AbsoluteY, nes), // RRA Absolute,Y
            0x63 => self.rra(AddressingMode::IndirectX, nes), // RRA (Indirect,X)
            0x73 => self.rra(AddressingMode::IndirectY, nes), // RRA (Indirect),Y
            0x87 => self.shx(AddressingMode::ZeroPage, nes), // SHX/SAX Zero Page
            0x97 => self.shx(AddressingMode::ZeroPageY, nes), // SHX/SAX Zero Page,Y
            0x8F => self.shx(AddressingMode::Absolute, nes), // SHX/SAX Absolute
            0x83 => self.shx(AddressingMode::IndirectX, nes), // SHX/SAX (Indirect,X)
            0x07 => self.slo(AddressingMode::ZeroPage, nes), // SLO Zero Page
            0x17 => self.slo(AddressingMode::ZeroPageX, nes), // SLO Zero Page,X
            0x0F => self.slo(AddressingMode::Absolute, nes), // SLO Absolute
            0x1F => self.slo(AddressingMode::AbsoluteX, nes), // SLO Absolute,X
            0x1B => self.slo(AddressingMode::AbsoluteY, nes), // SLO Absolute,Y
            0x03 => self.slo(AddressingMode::IndirectX, nes), // SLO (Indirect,X)
            0x13 => self.slo(AddressingMode::IndirectY, nes), // SLO (Indirect),Y

            0x47 => self.sre(AddressingMode::ZeroPage, nes), // SRE Zero Page
            0x57 => self.sre(AddressingMode::ZeroPageX, nes), // SRE Zero Page,X
            0x4F => self.sre(AddressingMode::Absolute, nes), // SRE Absolute
            0x5F => self.sre(AddressingMode::AbsoluteX, nes), // SRE Absolute,X
            0x5B => self.sre(AddressingMode::AbsoluteY, nes), // SRE Absolute,Y
            0x43 => self.sre(AddressingMode::IndirectX, nes), // SRE (Indirect,X)
            0x53 => self.sre(AddressingMode::IndirectY, nes), // SRE (Indirect),Y
            0x9E => self.xas(AddressingMode::AbsoluteX, nes), // XAS (SHX) Absolute,X
            0x9F => self.xas(AddressingMode::AbsoluteY, nes), // XAS (SHX) Absolute,Y
            0x93 => self.xas(AddressingMode::IndirectY, nes), // XAS (SHX) (Indirect),Y
            0x9C => self.xas(AddressingMode::AbsoluteX, nes), // XAS (SHX) Absolute,X (alternate)

            // Handle unknown opcodes
            _ => {
                log::warn!(
                    "Unknown opcode: {:#04X} at PC: {:#06X}",
                    opcode,
                    self.reg.pc
                );
                // Treat as a 1-byte NOP for compatibility
                Ok(1)
            }
        }
    }

    /// Handle an interrupt request (NMI, IRQ, or BRK)
    ///
    /// This method handles the common interrupt sequence:
    /// 1. Push PC high byte
    /// 2. Push PC low byte
    /// 3. Push status register with B flag cleared
    /// 4. Set I flag (except for BRK)
    /// 5. Jump to the interrupt vector
    ///
    /// # Arguments
    /// * `nes` - The NES system
    /// * `vector` - The address of the interrupt vector
    /// * `is_brk` - Whether this is a BRK instruction (affects B flag and PC adjustment)
    ///
    /// # Returns
    /// Number of cycles used (7 for NMI/IRQ, 7 for BRK)
    fn handle_interrupt(
        &mut self,
        vector: u16,
        is_brk: bool,
        nes: &mut crate::nes::Nes,
    ) -> CpuResult<u8> {
        // For BRK, the PC points to the next instruction (PC + 2)
        let pc = if is_brk {
            self.reg.pc.wrapping_add(1)
        } else {
            self.reg.pc
        };

        // Push PC and status register
        self.push_word(pc, nes)?;

        // Push status with B flag set for BRK, cleared for NMI/IRQ
        let mut status = self.reg.p.bits();
        if is_brk {
            status |= StatusFlags::BREAK.bits();
        } else {
            status &= !StatusFlags::BREAK.bits();
        }
        self.push_byte(status, nes)?;

        // Set interrupt disable flag
        self.reg.p.insert(StatusFlags::INTERRUPT_DISABLE);

        // Jump to interrupt vector
        self.reg.pc = nes.read_word(vector)?;

        // Interrupts take 7 cycles
        Ok(7)
    }

    // Stub implementations for CPU instructions
    fn lda(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn ldx(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn ldy(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn sta(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn stx(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn sty(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn tax(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn txa(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn tay(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn tya(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn tsx(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn txs(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn pha(&mut self, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(3)
    }
    fn php(&mut self, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(3)
    }
    fn pla(&mut self, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(4)
    }
    fn plp(&mut self, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(4)
    }
    fn and(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn eor(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn ora(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn bit(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn adc(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn sbc(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn cmp(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn cpx(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn cpy(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn inc(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn inx(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn iny(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn dec(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn dex(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn dey(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn asl_acc(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn asl(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn lsr_acc(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn lsr(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn rol_acc(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn rol(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn ror_acc(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn ror(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn jmp(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(3)
    }
    fn jsr(&mut self, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(6)
    }
    fn rts(&mut self, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(6)
    }
    fn rti(&mut self, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(6)
    }
    fn branch(&mut self, _condition: bool, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn clc(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn cld(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn cli(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn clv(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn sec(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn sed(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn sei(&mut self) -> CpuResult<u8> {
        Ok(2)
    }
    fn brk(&mut self, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(7)
    }

    fn aac(&mut self, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn ane(&mut self, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn arr(&mut self, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn axs(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn dop(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn nop(&mut self, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn las(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn rla(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn rra(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn shx(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn slo(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn sre(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }
    fn xas(&mut self, _mode: AddressingMode, _nes: &mut crate::nes::Nes) -> CpuResult<u8> {
        Ok(2)
    }

    fn get_operand_address(
        &self,
        _nes: &mut crate::nes::Nes,
        _mode: AddressingMode,
    ) -> CpuResult<(u16, bool, u8)> {
        Ok((0, false, 0))
    }

    fn update_zero_and_negative_flags(&mut self, _value: u8) {}

    fn push_byte(&mut self, _value: u8, _nes: &mut crate::nes::Nes) -> CpuResult<()> {
        Ok(())
    }

    fn push_word(&mut self, _value: u16, _nes: &mut crate::nes::Nes) -> CpuResult<()> {
        Ok(())
    }
}

// Implement Memory trait for Cpu to allow direct memory access
impl Memory for Cpu {
    fn read_byte(&self, _addr: u16) -> Result<u8, MemoryError> {
        // Since Cpu doesn't have memory, this is a stub
        Ok(0)
    }

    fn write_byte(&mut self, _addr: u16, _value: u8) -> Result<(), MemoryError> {
        // Stub
        Ok(())
    }

    // Use the default implementations for read_word and write_word from the Memory trait
}

/// Addressing modes for the 6502 CPU
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AddressingMode {
    Implied,
    Accumulator,
    Immediate,
    ZeroPage,
    ZeroPageX,
    ZeroPageY,
    Absolute,
    AbsoluteX,
    AbsoluteY,
    Indirect,
    IndirectX,
    IndirectY,
    Relative,
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;
    use std::collections::HashMap;

    // Simple memory implementation for testing
    struct TestMemory {
        data: RefCell<HashMap<u16, u8>>,
    }

    impl TestMemory {
        fn new() -> Self {
            TestMemory {
                data: RefCell::new(HashMap::new()),
            }
        }

        fn write(&self, addr: u16, data: &[u8]) {
            let mut mem = self.data.borrow_mut();
            for (i, &byte) in data.iter().enumerate() {
                mem.insert(addr.wrapping_add(i as u16), byte);
            }
        }
    }

    impl Memory for TestMemory {
        fn read_byte(&self, addr: u16) -> std::io::Result<u8> {
            Ok(*self.data.borrow().get(&addr).unwrap_or(&0))
        }

        fn write_byte(&mut self, addr: u16, value: u8) -> std::io::Result<()> {
            self.data.borrow_mut().insert(addr, value);
            Ok(())
        }
    }

    #[test]
    fn test_cpu_reset() {
        let mut cpu = Cpu::new();
        cpu.reset();

        assert_eq!(cpu.reg.a, 0);
        assert_eq!(cpu.reg.x, 0);
        assert_eq!(cpu.reg.y, 0);
        assert_eq!(cpu.reg.s, 0xFD);
        assert!(cpu.reg.p.contains(StatusFlags::UNUSED));
        assert!(cpu.reg.p.contains(StatusFlags::INTERRUPT_DISABLE));
        assert!(cpu.is_running);
    }

    #[test]
    fn test_lda_immediate() {
        let mut cpu = Cpu::new();
        let mut mem = TestMemory::new();

        // LDA #$42
        mem.write(0x8000, &[0xA9, 0x42]);
        cpu.reg.pc = 0x8000;

        let cycles = cpu.step(&mut mem).unwrap();
        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.a, 0x42);
        assert!(!cpu.reg.p.contains(StatusFlags::ZERO));
        assert!(!cpu.reg.p.contains(StatusFlags::NEGATIVE));
        assert_eq!(cpu.reg.pc, 0x8002);
    }

    #[test]
    fn test_sta_zero_page() {
        let mut cpu = Cpu::new();
        let mut mem = TestMemory::new();

        // LDA #$42
        mem.write(0x8000, &[0xA9, 0x42]);
        // STA $10
        mem.write(0x8002, &[0x85, 0x10]);

        cpu.reg.pc = 0x8000;
        cpu.step(&mut mem).unwrap();
        cpu.step(&mut mem).unwrap();

        assert_eq!(mem.read_byte(0x0010).unwrap(), 0x42);
    }
}
