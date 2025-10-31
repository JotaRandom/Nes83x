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

use super::registers::Registers;
use crate::memory::{self, Memory, MemoryError};
use std::fmt;

mod memory;
pub use memory::CpuMemory;

mod status_flags;
mod unofficial_ops;
#[cfg(feature = "unofficial_ops")]
mod unofficial_ops_consolidated;

pub use self::status_flags::StatusFlags;

/// Represents the result of a CPU operation
pub type CpuResult<T> = Result<T, CpuError>;

/// A wrapper that implements Memory for the CPU
/// This is used to provide a consistent interface for memory access
#[derive(Debug)]
pub struct CpuMemory<'a> {
    memory: &'a mut dyn Memory,
}

impl<'a> CpuMemory<'a> {
    /// Create a new CpuMemory that wraps the given Memory
    pub fn new(memory: &'a mut dyn Memory) -> Self {
        Self { memory }
    }
    
    /// Get a mutable reference to the underlying memory
    pub fn memory_mut(&mut self) -> &mut dyn Memory {
        self.memory
    }
}

impl Memory for CpuMemory<'_> {
    fn read_byte(&self, addr: u16) -> Result<u8, MemoryError> {
        self.memory.read_byte(addr)
    }

    fn write_byte(&mut self, addr: u16, value: u8) -> Result<(), MemoryError> {
        self.memory.write_byte(addr, value)
    }
    
    fn read_word(&self, addr: u16) -> Result<u16, MemoryError> {
        self.memory.read_word(addr)
    }
    
    fn write_word(&mut self, addr: u16, value: u16) -> Result<(), MemoryError> {
        self.memory.write_word(addr, value)
    }
}

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
pub struct Cpu<M: Memory> {
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
    /// Memory bus reference
    memory: CpuMemory,
}

use crate::nes::utils::MemoryError;

use std::fmt;

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

impl std::error::Error for CpuError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            CpuError::MemoryError(e) => Some(e),
            CpuError::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl<M: Memory> Cpu<M> {
    /// Creates a new CPU instance with default values
    ///
    /// Initializes the CPU to its power-on state:
    /// - All registers set to zero
    /// - Stack pointer initialized to 0xFD (typical boot value)
    /// - Status flags set to 0x34 (Interrupt Disable and Break flags set)
    /// - Program counter will be set by the reset vector
    /// - All interrupt flags cleared
    /// - CPU marked as not running
    pub fn new(memory: M) -> Self {
        let memory = CpuMemory::new(Box::new(memory));
        Cpu {
            reg: Registers {
                a: 0,
                x: 0,
                y: 0,
                s: 0xFD, // Default stack pointer
                pc: 0, // Will be set by the NES
                p: StatusFlags::UNUSED | StatusFlags::INTERRUPT_DISABLE,
            },
            cycles: 0,
            nmi_pending: false,
            irq_pending: false,
            is_running: false,
            memory,
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
    /// * `memory` - A mutable reference to the memory interface
    ///
    /// # Returns
    /// The number of cycles the instruction took to execute
    ///
    /// # Errors
    /// Returns `CpuError` if an invalid opcode is encountered or memory access fails
    pub fn step(&mut self) -> Result<u32, CpuError> {
        let memory = self.memory.memory_mut();
        if !self.is_running {
            return Err(CpuError::InvalidState);
        }

        let mut cycles: u32 = 0;

        // Handle NMI (highest priority)
        if self.nmi_pending {
            self.nmi_pending = false;
            cycles += self.handle_interrupt(memory, 0xFFFA, false)? as u32;
            return Ok(cycles);
        }

        // Handle IRQ (if not masked)
        if self.irq_pending && !self.reg.p.contains(StatusFlags::INTERRUPT_DISABLE) {
            self.irq_pending = false;
            cycles += self.handle_interrupt(memory, 0xFFFE, false)? as u32;
            return Ok(cycles);
        }

        // Fetch the next opcode
        let opcode = memory.read_byte(self.reg.pc)?;
        self.reg.pc = self.reg.pc.wrapping_add(1);

        // Execute the instruction
        cycles += self.execute_instruction(opcode, memory)? as u32;

        Ok(cycles)
    }
    
    /// Execute a single CPU instruction (internal implementation)
    /// 
    /// # Arguments
    /// * `opcode` - The opcode to execute
    /// * `memory` - A mutable reference to the memory interface
    /// 
    /// # Returns
    /// The number of cycles the instruction took to execute, or an error if the opcode is invalid
    fn execute_instruction(&mut self, opcode: u8) -> CpuResult<u8> {
        let memory = self.memory.memory_mut();
        match opcode {
            // Unofficial opcodes (enabled with the 'unofficial_ops' feature)
            #[cfg(feature = "unofficial_ops")]
            0x0B | 0x2B => self.aac(memory),  // AAC (ANC)
            #[cfg(feature = "unofficial_ops")]
            0x8B => self.ane(memory),         // ANE (XAA)
            #[cfg(feature = "unofficial_ops")]
            0x6B => self.arr(memory),         // ARR
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
                self.lax(mode, memory)  // LAX (LDA + TAX)
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
                self.rla(mode, memory)  // RLA
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
                self.sre(mode, memory)  // SRE (LSE)
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
                self.rra(mode, memory)  // RRA
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
                self.dcp(mode, memory)  // DCP
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
                self.isc(mode, memory)  // ISC (ISB)
            }
            // Load/Store Operations
            0xA9 => self.lda(AddressingMode::Immediate, memory),    // LDA Immediate
            0xA5 => self.lda(AddressingMode::ZeroPage, memory),     // LDA Zero Page
            0xB5 => self.lda(AddressingMode::ZeroPageX, memory),    // LDA Zero Page,X
            0xAD => self.lda(AddressingMode::Absolute, memory),     // LDA Absolute
            0xBD => self.lda(AddressingMode::AbsoluteX, memory),    // LDA Absolute,X
            0xB9 => self.lda(AddressingMode::AbsoluteY, memory),    // LDA Absolute,Y
            0xA1 => self.lda(AddressingMode::IndirectX, memory),    // LDA (Indirect,X)
            0xB1 => self.lda(AddressingMode::IndirectY, memory),    // LDA (Indirect),Y
            
            0xA2 => self.ldx(AddressingMode::Immediate, memory),    // LDX Immediate
            0xA6 => self.ldx(AddressingMode::ZeroPage, memory),     // LDX Zero Page
            0xB6 => self.ldx(AddressingMode::ZeroPageY, memory),    // LDX Zero Page,Y
            0xAE => self.ldx(AddressingMode::Absolute, memory),     // LDX Absolute
            0xBE => self.ldx(AddressingMode::AbsoluteY, memory),    // LDX Absolute,Y
            
            0xA0 => self.ldy(AddressingMode::Immediate, memory),    // LDY Immediate
            0xA4 => self.ldy(AddressingMode::ZeroPage, memory),     // LDY Zero Page
            0xB4 => self.ldy(AddressingMode::ZeroPageX, memory),    // LDY Zero Page,X
            0xAC => self.ldy(AddressingMode::Absolute, memory),     // LDY Absolute
            0xBC => self.ldy(AddressingMode::AbsoluteX, memory),    // LDY Absolute,X
            
            0x85 => self.sta(AddressingMode::ZeroPage, memory),     // STA Zero Page
            0x95 => self.sta(AddressingMode::ZeroPageX, memory),    // STA Zero Page,X
            0x8D => self.sta(AddressingMode::Absolute, memory),     // STA Absolute
            0x9D => self.sta(AddressingMode::AbsoluteX, memory),    // STA Absolute,X
            0x99 => self.sta(AddressingMode::AbsoluteY, memory),    // STA Absolute,Y
            0x81 => self.sta(AddressingMode::IndirectX, memory),    // STA (Indirect,X)
            0x91 => self.sta(AddressingMode::IndirectY, memory),    // STA (Indirect),Y
            
            0x86 => self.stx(AddressingMode::ZeroPage, memory),     // STX Zero Page
            0x96 => self.stx(AddressingMode::ZeroPageY, memory),    // STX Zero Page,Y
            0x8E => self.stx(AddressingMode::Absolute, memory),     // STX Absolute
            
            0x84 => self.sty(AddressingMode::ZeroPage, memory),     // STY Zero Page
            0x94 => self.sty(AddressingMode::ZeroPageX, memory),    // STY Zero Page,X
            0x8C => self.sty(AddressingMode::Absolute, memory),     // STY Absolute
            
            // Register Transfers
            0xAA => self.tax(),                                     // TAX
            0x8A => self.txa(),                                     // TXA
            0xA8 => self.tay(),                                     // TAY
            0x98 => self.tya(),                                     // TYA
            0xBA => self.tsx(),                                     // TSX
            0x9A => self.txs(),                                     // TXS
            
            // Stack Operations
            0x48 => self.pha(memory),                               // PHA
            0x08 => self.php(memory),                               // PHP
            0x68 => self.pla(memory),                               // PLA
            0x28 => self.plp(memory),                               // PLP
            
            // Logical Operations
            0x29 => self.and(AddressingMode::Immediate, memory),    // AND Immediate
            0x25 => self.and(AddressingMode::ZeroPage, memory),     // AND Zero Page
            0x35 => self.and(AddressingMode::ZeroPageX, memory),    // AND Zero Page,X
            0x2D => self.and(AddressingMode::Absolute, memory),     // AND Absolute
            0x3D => self.and(AddressingMode::AbsoluteX, memory),    // AND Absolute,X
            0x39 => self.and(AddressingMode::AbsoluteY, memory),    // AND Absolute,Y
            0x21 => self.and(AddressingMode::IndirectX, memory),    // AND (Indirect,X)
            0x31 => self.and(AddressingMode::IndirectY, memory),    // AND (Indirect),Y
            
            0x49 => self.eor(AddressingMode::Immediate, memory),    // EOR Immediate
            0x45 => self.eor(AddressingMode::ZeroPage, memory),     // EOR Zero Page
            0x55 => self.eor(AddressingMode::ZeroPageX, memory),    // EOR Zero Page,X
            0x4D => self.eor(AddressingMode::Absolute, memory),     // EOR Absolute
            0x5D => self.eor(AddressingMode::AbsoluteX, memory),    // EOR Absolute,X
            0x59 => self.eor(AddressingMode::AbsoluteY, memory),    // EOR Absolute,Y
            0x41 => self.eor(AddressingMode::IndirectX, memory),    // EOR (Indirect,X)
            0x51 => self.eor(AddressingMode::IndirectY, memory),    // EOR (Indirect),Y
            
            0x09 => self.ora(AddressingMode::Immediate, memory),    // ORA Immediate
            0x05 => self.ora(AddressingMode::ZeroPage, memory),     // ORA Zero Page
            0x15 => self.ora(AddressingMode::ZeroPageX, memory),    // ORA Zero Page,X
            0x0D => self.ora(AddressingMode::Absolute, memory),     // ORA Absolute
            0x1D => self.ora(AddressingMode::AbsoluteX, memory),    // ORA Absolute,X
            0x19 => self.ora(AddressingMode::AbsoluteY, memory),    // ORA Absolute,Y
            0x01 => self.ora(AddressingMode::IndirectX, memory),    // ORA (Indirect,X)
            
            0x24 => self.bit(AddressingMode::ZeroPage, memory),     // BIT Zero Page
            0x2C => self.bit(AddressingMode::Absolute, memory),     // BIT Absolute
            
            // Arithmetic Operations
            0x69 => self.adc(AddressingMode::Immediate, memory),    // ADC Immediate
            0x65 => self.adc(AddressingMode::ZeroPage, memory),     // ADC Zero Page
            0x75 => self.adc(AddressingMode::ZeroPageX, memory),    // ADC Zero Page,X
            0x6D => self.adc(AddressingMode::Absolute, memory),     // ADC Absolute
            0x7D => self.adc(AddressingMode::AbsoluteX, memory),    // ADC Absolute,X
            0x79 => self.adc(AddressingMode::AbsoluteY, memory),    // ADC Absolute,Y
            0x61 => self.adc(AddressingMode::IndirectX, memory),    // ADC (Indirect,X)
            0x71 => self.adc(AddressingMode::IndirectY, memory),    // ADC (Indirect),Y
            
            0xE9 => self.sbc(AddressingMode::Immediate, memory),    // SBC Immediate
            0xE5 => self.sbc(AddressingMode::ZeroPage, memory),     // SBC Zero Page
            0xF5 => self.sbc(AddressingMode::ZeroPageX, memory),    // SBC Zero Page,X
            0xED => self.sbc(AddressingMode::Absolute, memory),     // SBC Absolute
            0xFD => self.sbc(AddressingMode::AbsoluteX, memory),    // SBC Absolute,X
            0xF9 => self.sbc(AddressingMode::AbsoluteY, memory),    // SBC Absolute,Y
            0xE1 => self.sbc(AddressingMode::IndirectX, memory),    // SBC (Indirect,X)
            0xF1 => self.sbc(AddressingMode::IndirectY, memory),    // SBC (Indirect),Y
            
            0xC9 => self.cmp(AddressingMode::Immediate, memory),    // CMP Immediate
            0xC5 => self.cmp(AddressingMode::ZeroPage, memory),     // CMP Zero Page
            0xD5 => self.cmp(AddressingMode::ZeroPageX, memory),    // CMP Zero Page,X
            0xCD => self.cmp(AddressingMode::Absolute, memory),     // CMP Absolute
            0xDD => self.cmp(AddressingMode::AbsoluteX, memory),    // CMP Absolute,X
            0xD9 => self.cmp(AddressingMode::AbsoluteY, memory),    // CMP Absolute,Y
            0xC1 => self.cmp(AddressingMode::IndirectX, memory),    // CMP (Indirect,X)
            0xD1 => self.cmp(AddressingMode::IndirectY, memory),    // CMP (Indirect),Y
            
            0xE0 => self.cpx(AddressingMode::Immediate, memory),    // CPX Immediate
            0xE4 => self.cpx(AddressingMode::ZeroPage, memory),     // CPX Zero Page
            0xEC => self.cpx(AddressingMode::Absolute, memory),     // CPX Absolute
            
            0xC0 => self.cpy(AddressingMode::Immediate, memory),    // CPY Immediate
            0xC4 => self.cpy(AddressingMode::ZeroPage, memory),     // CPY Zero Page
            0xCC => self.cpy(AddressingMode::Absolute, memory),     // CPY Absolute
            
            // Increments & Decrements
            0xE6 => self.inc(AddressingMode::ZeroPage, memory),     // INC Zero Page
            0xF6 => self.inc(AddressingMode::ZeroPageX, memory),    // INC Zero Page,X
            0xEE => self.inc(AddressingMode::Absolute, memory),     // INC Absolute
            0xFE => self.inc(AddressingMode::AbsoluteX, memory),    // INC Absolute,X
            
            0xE8 => self.inx(),                                     // INX
            0xC8 => self.iny(),                                     // INY
            
            0xC6 => self.dec(AddressingMode::ZeroPage, memory),     // DEC Zero Page
            0xD6 => self.dec(AddressingMode::ZeroPageX, memory),    // DEC Zero Page,X
            0xCE => self.dec(AddressingMode::Absolute, memory),     // DEC Absolute
            0xDE => self.dec(AddressingMode::AbsoluteX, memory),    // DEC Absolute,X
            
            0xCA => self.dex(),                                     // DEX
            0x88 => self.dey(),                                     // DEY
            
            // Shifts
            0x0A => self.asl_acc(),                                 // ASL Accumulator
            0x06 => self.asl(AddressingMode::ZeroPage, memory),     // ASL Zero Page
            0x16 => self.asl(AddressingMode::ZeroPageX, memory),    // ASL Zero Page,X
            0x0E => self.asl(AddressingMode::Absolute, memory),     // ASL Absolute
            0x1E => self.asl(AddressingMode::AbsoluteX, memory),    // ASL Absolute,X
            
            0x4A => self.lsr_acc(),                                 // LSR Accumulator
            0x46 => self.lsr(AddressingMode::ZeroPage, memory),     // LSR Zero Page
            0x56 => self.lsr(AddressingMode::ZeroPageX, memory),    // LSR Zero Page,X
            0x4E => self.lsr(AddressingMode::Absolute, memory),     // LSR Absolute
            0x5E => self.lsr(AddressingMode::AbsoluteX, memory),    // LSR Absolute,X
            
            0x2A => self.rol_acc(),                                 // ROL Accumulator
            0x26 => self.rol(AddressingMode::ZeroPage, memory),     // ROL Zero Page
            0x36 => self.rol(AddressingMode::ZeroPageX, memory),    // ROL Zero Page,X
            0x2E => self.rol(AddressingMode::Absolute, memory),     // ROL Absolute
            0x3E => self.rol(AddressingMode::AbsoluteX, memory),    // ROL Absolute,X
            
            0x6A => self.ror_acc(),                                 // ROR Accumulator
            0x66 => self.ror(AddressingMode::ZeroPage, memory),     // ROR Zero Page
            0x76 => self.ror(AddressingMode::ZeroPageX, memory),    // ROR Zero Page,X
            0x6E => self.ror(AddressingMode::Absolute, memory),     // ROR Absolute
            0x7E => self.ror(AddressingMode::AbsoluteX, memory),    // ROR Absolute,X
            
            // Jumps & Calls
            0x4C => self.jmp(AddressingMode::Absolute, memory),     // JMP Absolute
            0x6C => self.jmp(AddressingMode::Indirect, memory),     // JMP Indirect
            0x20 => self.jsr(memory),                               // JSR
            0x60 => self.rts(memory),                               // RTS
            0x40 => self.rti(memory),                               // RTI
            
            // Branches
            0x90 => self.branch(!self.reg.p.contains(StatusFlags::CARRY), memory),     // BCC
            0xB0 => self.branch(self.reg.p.contains(StatusFlags::CARRY), memory),      // BCS
            0xF0 => self.branch(self.reg.p.contains(StatusFlags::ZERO), memory),       // BEQ
            0x30 => self.branch(self.reg.p.contains(StatusFlags::NEGATIVE), memory),   // BMI
            0xD0 => self.branch(!self.reg.p.contains(StatusFlags::ZERO), memory),      // BNE
            0x10 => self.branch(!self.reg.p.contains(StatusFlags::NEGATIVE), memory),  // BPL
            0x50 => self.branch(!self.reg.p.contains(StatusFlags::OVERFLOW), memory),  // BVC
            0x70 => self.branch(self.reg.p.contains(StatusFlags::OVERFLOW), memory),   // BVS
            
            // Status Flag Changes
            0x18 => self.clc(),                                                       // CLC
            0xD8 => self.cld(),                                                       // CLD
            0x58 => self.cli(),                                                       // CLI
            0xB8 => self.clv(),                                                       // CLV
            0x38 => self.sec(),                                                       // SEC
            0xF8 => self.sed(),                                                       // SED
            0x78 => self.sei(),                                                       // SEI
            
            // System Functions
            0x00 => self.brk(memory),                                                 // BRK
            0xEA => Ok(2),                                                            // NOP
            
            // Unofficial/Illegal Opcodes (stubs for now)
            0x0B | 0x2B => self.aac(memory),                                      // AAC (ANC)
            0x8B => self.ane(memory),                                                 // ANE (XAA)
            0x6B => self.arr(memory),                                                 // ARR
            0xCB => self.axs(AddressingMode::Immediate, memory),                      // AXS (SBX)
            0xE2 => self.dop(AddressingMode::Immediate, memory),                      // DOP (NOP)
            0x04 | 0x44 | 0x64 => self.nop(memory),    // NOP Zero Page
            0x14 | 0x34 | 0x54 | 0x74 | 0xD4 | 0xF4 => self.nop(memory), // NOP Zero Page,X
            0x80 | 0x82 | 0x89 | 0xC2 | 0xE2 => self.nop(memory), // NOP Immediate
            0x0C => self.nop(memory),                  // NOP Absolute
            0x1C | 0x3C | 0x5C | 0x7C | 0xDC | 0xFC => self.nop(memory), // NOP Absolute,X
            0xA7 => self.las(AddressingMode::ZeroPage, memory),    // LAS (Unofficial)
            0x1A | 0x3A | 0x5A | 0x7A | 0xDA | 0xFA => self.nop(memory), // NOP Implied
            0x27 => self.rla(AddressingMode::ZeroPage, memory),                      // RLA Zero Page
            0x37 => self.rla(AddressingMode::ZeroPageX, memory),                     // RLA Zero Page,X
            0x2F => self.rla(AddressingMode::Absolute, memory),                      // RLA Absolute
            0x3F => self.rla(AddressingMode::AbsoluteX, memory),                     // RLA Absolute,X
            0x3B => self.rla(AddressingMode::AbsoluteY, memory),                     // RLA Absolute,Y
            0x23 => self.rla(AddressingMode::IndirectX, memory),                     // RLA (Indirect,X)
            0x33 => self.rla(AddressingMode::IndirectY, memory),                     // RLA (Indirect),Y
            0x67 => self.rra(AddressingMode::ZeroPage, memory),                      // RRA Zero Page
            0x77 => self.rra(AddressingMode::ZeroPageX, memory),                     // RRA Zero Page,X
            0x6F => self.rra(AddressingMode::Absolute, memory),                      // RRA Absolute
            0x7F => self.rra(AddressingMode::AbsoluteX, memory),                     // RRA Absolute,X
            0x7B => self.rra(AddressingMode::AbsoluteY, memory),                     // RRA Absolute,Y
            0x63 => self.rra(AddressingMode::IndirectX, memory),                     // RRA (Indirect,X)
            0x73 => self.rra(AddressingMode::IndirectY, memory),                     // RRA (Indirect),Y
            0x87 => self.shx(AddressingMode::ZeroPage, memory),                      // SHX/SAX Zero Page
            0x97 => self.shx(AddressingMode::ZeroPageY, memory),                     // SHX/SAX Zero Page,Y
            0x8F => self.shx(AddressingMode::Absolute, memory),                      // SHX/SAX Absolute
            0x83 => self.shx(AddressingMode::IndirectX, memory),                     // SHX/SAX (Indirect,X)
            0x07 => self.slo(AddressingMode::ZeroPage, memory),                      // SLO Zero Page
            0x17 => self.slo(AddressingMode::ZeroPageX, memory),                     // SLO Zero Page,X
            0x0F => self.slo(AddressingMode::Absolute, memory),                      // SLO Absolute
            0x1F => self.slo(AddressingMode::AbsoluteX, memory),                     // SLO Absolute,X
            0x1B => self.slo(AddressingMode::AbsoluteY, memory),                     // SLO Absolute,Y
            0x03 => self.slo(AddressingMode::IndirectX, memory),                     // SLO (Indirect,X)
            0x13 => self.slo(AddressingMode::IndirectY, memory),                     // SLO (Indirect),Y
            
            0x47 => self.sre(AddressingMode::ZeroPage, memory),                      // SRE Zero Page
            0x57 => self.sre(AddressingMode::ZeroPageX, memory),                     // SRE Zero Page,X
            0x4F => self.sre(AddressingMode::Absolute, memory),                      // SRE Absolute
            0x5F => self.sre(AddressingMode::AbsoluteX, memory),                     // SRE Absolute,X
            0x5B => self.sre(AddressingMode::AbsoluteY, memory),                     // SRE Absolute,Y
            0x43 => self.sre(AddressingMode::IndirectX, memory),                     // SRE (Indirect,X)
            0x53 => self.sre(AddressingMode::IndirectY, memory),                     // SRE (Indirect),Y
            0x9E => self.xas(AddressingMode::AbsoluteX, memory),                     // XAS (SHX) Absolute,X
            0x9F => self.xas(AddressingMode::AbsoluteY, memory),                     // XAS (SHX) Absolute,Y
            0x93 => self.xas(AddressingMode::IndirectY, memory),                     // XAS (SHX) (Indirect),Y
            0x9C => self.xas(AddressingMode::AbsoluteX, memory),                     // XAS (SHX) Absolute,X (alternate)
            
            // Handle unknown opcodes
            _ => {
                log::warn!("Unknown opcode: {:#04X} at PC: {:#06X}", opcode, self.reg.pc);
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
    /// * `memory` - The memory interface
    /// * `vector` - The address of the interrupt vector
    /// * `is_brk` - Whether this is a BRK instruction (affects B flag and PC adjustment)
    /// 
    /// # Returns
    /// Number of cycles used (7 for NMI/IRQ, 7 for BRK)
    fn handle_interrupt(&mut self, vector: u16, is_brk: bool) -> CpuResult<u8> {
        let memory = self.memory.memory_mut();
        // For BRK, the PC points to the next instruction (PC + 2)
        let pc = if is_brk {
            self.reg.pc.wrapping_add(1)
        } else {
            self.reg.pc
        };
        
        // Push PC and status to stack (high byte first)
        self.push_byte(memory, (pc >> 8) as u8)?;
        self.push_byte(memory, pc as u8)?;
        
        // Push status with Break flag set for BRK, cleared otherwise
        let mut status = self.reg.p;
        if is_brk {
            status.insert(StatusFlags::BREAK);
        } else {
            status.remove(StatusFlags::BREAK);
        }
        // Always set the unused bit when pushing to stack
        status.insert(StatusFlags::UNUSED);
        self.push_byte(memory, status.bits())?;
        
        // Set interrupt disable flag (except for BRK which already has it set)
        if !is_brk {
            self.reg.p.insert(StatusFlags::INTERRUPT_DISABLE);
        }
        
        // Read the new PC from the interrupt vector
        self.reg.pc = memory.read_word(vector)?;
        
        // Interrupts take 7 cycles
        Ok(7)
    }
    
    // =============================================
    // Addressing Modes
    // =============================================
    
    /// Get the address for the specified addressing mode
    /// Returns (address, page_crossed, extra_cycles)
    /// 
    /// - address: The resolved memory address
    /// - page_crossed: Whether a page boundary was crossed (for cycle counting)
    /// - extra_cycles: Number of additional cycles needed (0 or 1 for page crossing)
    fn get_operand_address(&mut self, mode: AddressingMode) -> CpuResult<(u16, bool, u8)> {
        let memory = self.memory.memory_mut();
        match mode {
            AddressingMode::Implied => Ok((0, false, 0)),
            AddressingMode::Accumulator => Ok((0, false, 0)),
            AddressingMode::Immediate => {
                let addr = self.reg.pc;
                self.reg.pc = self.reg.pc.wrapping_add(1);
                Ok((addr, false, 0))
            },
            AddressingMode::ZeroPage => {
                let addr = memory.read_byte(self.reg.pc)? as u16;
                self.reg.pc = self.reg.pc.wrapping_add(1);
                Ok((addr, false, 0))
            },
            AddressingMode::ZeroPageX => {
                let base = memory.read_byte(self.reg.pc)? as u16;
                self.reg.pc = self.reg.pc.wrapping_add(1);
                let addr = base.wrapping_add(self.reg.x as u16) & 0x00FF; // Wrap around in zero page
                // No page crossing possible in zero page
                Ok((addr, false, 0))
            },
            AddressingMode::ZeroPageY => {
                let base = memory.read_byte(self.reg.pc)? as u16;
                self.reg.pc = self.reg.pc.wrapping_add(1);
                let addr = base.wrapping_add(self.reg.y as u16) & 0x00FF; // Wrap around in zero page
                // No page crossing possible in zero page
                Ok((addr, false, 0))
            },
            AddressingMode::Absolute => {
                let addr = memory.read_word(self.reg.pc)?;
                self.reg.pc = self.reg.pc.wrapping_add(2);
                Ok((addr, false, 0))
            },
            AddressingMode::AbsoluteX => {
                let base = memory.read_word(self.reg.pc)?;
                self.reg.pc = self.reg.pc.wrapping_add(2);
                let addr = base.wrapping_add(self.reg.x as u16);
                // Page crossing occurs if the high byte changes
                let page_crossed = (base & 0xFF00) != (addr & 0xFF00);
                // Return 1 extra cycle if page was crossed
                Ok((addr, page_crossed, if page_crossed { 1 } else { 0 }))
            },
            AddressingMode::AbsoluteY => {
                let base = memory.read_word(self.reg.pc)?;
                self.reg.pc = self.reg.pc.wrapping_add(2);
                let addr = base.wrapping_add(self.reg.y as u16);
                // Page crossing occurs if the high byte changes
                let page_crossed = (base & 0xFF00) != (addr & 0xFF00);
                // Return 1 extra cycle if page was crossed
                Ok((addr, page_crossed, if page_crossed { 1 } else { 0 }))
            },
            AddressingMode::Indirect => {
                let ptr = memory.read_word(self.reg.pc)?;
                self.reg.pc = self.reg.pc.wrapping_add(2);
                
                // Handle 6502 page boundary bug
                let lo = memory.read_byte(ptr)? as u16;
                let hi_ptr = if (ptr & 0x00FF) == 0x00FF {
                    ptr & 0xFF00
                } else {
                    ptr.wrapping_add(1)
                };
                let hi = memory.read_byte(hi_ptr)? as u16;
                
                Ok(((hi << 8) | lo, false, 0))
            },
            AddressingMode::IndirectX => {
                let base = memory.read_byte(self.reg.pc)? as u16;
                self.reg.pc = self.reg.pc.wrapping_add(1);
                // Wrap around in zero page for the pointer
                let ptr = base.wrapping_add(self.reg.x as u16) & 0x00FF;
                // Read the low byte
                let lo = memory.read_byte(ptr)? as u16;
                // Read the high byte (wrap around in zero page)
                let hi = memory.read_byte((ptr + 1) & 0x00FF)? as u16;
                let addr = (hi << 8) | lo;
                // No page crossing penalty for indirect X
                Ok((addr, false, 0))
            },
            AddressingMode::IndirectY => {
                let base = memory.read_byte(self.reg.pc)? as u16;
                self.reg.pc = self.reg.pc.wrapping_add(1);
                // Read the 16-bit address from zero page (wraps around in page 0)
                let lo = memory.read_byte(base & 0x00FF)? as u16;
                let hi = memory.read_byte((base + 1) & 0x00FF)? as u16;
                let deref = (hi << 8) | lo;
                let addr = deref.wrapping_add(self.reg.y as u16);
                // Page crossing occurs if the addition of Y crosses a page boundary
                let page_crossed = (deref & 0xFF00) != (addr & 0xFF00);
                // Return 1 extra cycle if page was crossed
                Ok((addr, page_crossed, if page_crossed { 1 } else { 0 }))
            },
            AddressingMode::Relative => {
                let offset = memory.read_byte(self.reg.pc)? as i8;
                self.reg.pc = self.reg.pc.wrapping_add(1);
                let addr = self.reg.pc.wrapping_add(offset as u16);
                Ok((addr, false, 0))
            },
        }
    }
    
    // =============================================
    // Flag Operations
    // =============================================
    
    /// Updates the Zero and Negative flags based on the given value
    /// 
    /// # Arguments
    /// * `value` - The value to check for zero/negative
    pub(crate) fn update_zero_and_negative_flags(&mut self, value: u8) {
        // Set zero flag if the result is zero
        self.reg.p.set(StatusFlags::ZERO, value == 0);
        
        // Set negative flag if bit 7 is set
        self.reg.p.set(StatusFlags::NEGATIVE, (value & 0x80) != 0);
    }
    
    // =============================================
    // Stack Operations
    // =============================================
    
    fn push_byte(&mut self, value: u8) -> CpuResult<()> {
        let memory = self.memory.memory_mut();
        memory.write_byte(0x0100 | (self.reg.s as u16), value)?;
        self.reg.s = self.reg.s.wrapping_sub(1);
        Ok(())
    }
    
    fn push_word(&mut self, value: u16) -> CpuResult<()> {
        self.push_byte((value >> 8) as u8)?;
        self.push_byte(value as u8)
    }
    
    fn pull_byte(&mut self) -> CpuResult<u8> {
        let memory = self.memory.memory_mut();
        self.reg.s = self.reg.s.wrapping_add(1);
        memory.read_byte(0x0100 | (self.reg.s as u16))
    }
    
    fn pull_word(&mut self) -> CpuResult<u16> {
        let lo = self.pull_byte()? as u16;
        let hi = self.pull_byte()? as u16;
        Ok((hi << 8) | lo)
    }
    
    /// ADC - Add with Carry
    /// 
    /// This instruction adds the contents of a memory location to the accumulator together with the carry bit.
    /// If overflow occurs the carry bit is set, this enables multiple byte addition to be performed.
    fn adc(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr)?;
        
        self.adc_impl(value)?;
        
        // Base cycles + extra cycles for page crossing
        let base_cycles = match mode {
            AddressingMode::Immediate => 2,
            AddressingMode::ZeroPage => 3,
            AddressingMode::ZeroPageX => 4,
            AddressingMode::Absolute => 4,
            AddressingMode::AbsoluteX | AddressingMode::AbsoluteY => 4,
            AddressingMode::IndirectX => 6,
            AddressingMode::IndirectY => 5,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        };
        
        Ok(base_cycles + extra_cycles as u8)
    }
    
    /// Internal implementation of ADC operation
    /// 
    /// This is a helper method that performs the actual ADC operation and updates the flags.
    /// It's used by both the regular ADC instruction and by other instructions that need to perform
    /// addition with carry (like SBC with the carry flag inverted).
    /// 
    /// # Arguments
    /// * `value` - The value to add to the accumulator
    /// 
    /// # Returns
    /// The number of cycles the operation took (1 for the internal operation)
    fn adc_impl(&mut self, value: u8) -> CpuResult<u8> {
        let carry = if self.reg.p.contains(StatusFlags::CARRY) { 1 } else { 0 };
        let sum = self.reg.a as u16 + value as u16 + carry;
        
        // Set carry flag if sum > 255
        self.reg.p.set(StatusFlags::CARRY, sum > 0xFF);
        
        // Set overflow flag if the sign of the result is different from both operands
        let result = sum as u8;
        let overflow = ((self.reg.a ^ result) & (value ^ result) & 0x80) != 0;
        self.reg.p.set(StatusFlags::OVERFLOW, overflow);
        
        // Update accumulator and flags
        self.reg.a = result;
        self.update_zero_and_negative_flags(self.reg.a);
        
        Ok(1)  // Base cycles (additional cycles handled by the caller)
    }
    
    
    /// SBC - Subtract with Carry
    fn sbc(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        // SBC is the same as ADC with the operand's bits inverted
        // and the carry flag treated as inverted
        let (addr, _page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = if matches!(mode, AddressingMode::Immediate) {
            addr as u8
        } else {
            memory.read_byte(addr)?
        };
        
        // Invert the bits of the value and use ADC logic
        let carry = if self.reg.p.contains(StatusFlags::CARRY) { 1 } else { 0 };
        let inverted_value = (!value).wrapping_add(carry);
        let sum = self.reg.a as u16 + inverted_value as u16;
        
        // Set carry flag if sum > 255 (same as ADC)
        self.reg.p.set(StatusFlags::CARRY, sum > 0xFF);
        
        // Set overflow flag if the sign of the result is different from both operands
        let result = sum as u8;
        let overflow = ((self.reg.a ^ result) & ((!value) ^ result) & 0x80) != 0;
        self.reg.p.set(StatusFlags::OVERFLOW, overflow);
        
        self.reg.a = result;
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Calculate base cycles based on addressing mode
        let base_cycles = match mode {
            AddressingMode::Immediate => 2,
            AddressingMode::ZeroPage => 3,
            AddressingMode::ZeroPageX => 4,
            AddressingMode::Absolute => 4,
            AddressingMode::AbsoluteX | AddressingMode::AbsoluteY => 4,
            AddressingMode::IndirectX => 6,
            AddressingMode::IndirectY => 5,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        };
        
        // Add extra cycle if page boundary was crossed
        Ok(base_cycles + extra_cycles as u8)
    }
    
    /// AND - Logical AND
    fn and(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = if matches!(mode, AddressingMode::Immediate) {
            addr as u8
        } else {
            memory.read_byte(addr)?
        };
        
        self.reg.a &= value;
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Base cycles + extra cycles for page crossing
        Ok(match mode {
            AddressingMode::Immediate => 2,
            AddressingMode::ZeroPage => 3,
            AddressingMode::ZeroPageX => 4,
            AddressingMode::Absolute => 4,
            AddressingMode::AbsoluteX | AddressingMode::AbsoluteY => 4 + extra_cycles,
            AddressingMode::IndirectX => 6,
            AddressingMode::IndirectY => 5 + extra_cycles,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    /// EOR - Exclusive OR
    fn eor(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = if matches!(mode, AddressingMode::Immediate) {
            addr as u8
        } else {
            memory.read_byte(addr)?
        };
        
        self.reg.a ^= value;
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Base cycles + extra cycles for page crossing
        Ok(match mode {
            AddressingMode::Immediate => 2,
            AddressingMode::ZeroPage => 3,
            AddressingMode::ZeroPageX => 4,
            AddressingMode::Absolute => 4,
            AddressingMode::AbsoluteX | AddressingMode::AbsoluteY => 4 + extra_cycles,
            AddressingMode::IndirectX => 6,
            AddressingMode::IndirectY => 5 + extra_cycles,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    /// ORA - Logical Inclusive OR
    fn ora(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = if matches!(mode, AddressingMode::Immediate) {
            addr as u8
        } else {
            memory.read_byte(addr)?
        };
        
        self.reg.a |= value;
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Base cycles + extra cycles for page crossing
        Ok(match mode {
            AddressingMode::Immediate => 2,
            AddressingMode::ZeroPage => 3,
            AddressingMode::ZeroPageX => 4,
            AddressingMode::Absolute => 4,
            AddressingMode::AbsoluteX | AddressingMode::AbsoluteY => 4 + extra_cycles,
            AddressingMode::IndirectX => 6,
            AddressingMode::IndirectY => 5 + extra_cycles,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    /// BIT - Bit Test
    fn bit(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _, _) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr)?;
        
        // Set zero flag if (A & value) == 0
        self.reg.p.set(StatusFlags::ZERO, (self.reg.a & value) == 0);
        
        // Set overflow flag to bit 6 of value
        self.reg.p.set(StatusFlags::OVERFLOW, (value & 0x40) != 0);
        
        // Set negative flag to bit 7 of value
        self.reg.p.set(StatusFlags::NEGATIVE, (value & 0x80) != 0);
        
        Ok(match mode {
            AddressingMode::ZeroPage => 3,
            AddressingMode::Absolute => 4,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    /// CMP - Compare Accumulator
    fn cmp(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        self.compare_register(mode, self.reg.a, memory)
    }
    
    /// CPX - Compare X Register
    fn cpx(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        self.compare_register(mode, self.reg.x, memory)
    }
    
    /// CPY - Compare Y Register
    fn cpy(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        self.compare_register(mode, self.reg.y, memory)
    }
    
    /// Common compare logic used by CMP, CPX, CPY
    fn compare_register(&mut self, mode: AddressingMode, register: u8, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _page_crossed, _extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = if matches!(mode, AddressingMode::Immediate) { addr as u8 } else { memory.read_byte(addr)? };
        
        // Compare is like SBC but doesn't store the result
        let result = register.wrapping_sub(value);
        
        // Set carry if register >= value
        self.reg.p.set(StatusFlags::CARRY, register >= value);
        
        // Update zero and negative flags based on the result
        self.update_zero_and_negative_flags(result);
        
        // Return cycle count based on addressing mode
        Ok(match mode {
            AddressingMode::Immediate => 2,
            AddressingMode::ZeroPage => 3,
            AddressingMode::Absolute => 4,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    // =============================================
    // Increment/Decrement Operations
    // =============================================
    
    /// INC - Increment Memory
    fn inc(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _, _) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr)?.wrapping_add(1);
        memory.write_byte(addr, value)?;
        self.update_zero_and_negative_flags(value);
        
        Ok(match mode {
            AddressingMode::ZeroPage => 5,
            AddressingMode::ZeroPageX => 6,
            AddressingMode::Absolute => 6,
            AddressingMode::AbsoluteX => 7,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    /// INX - Increment X Register
    fn inx(&mut self) -> CpuResult<u8> {
        self.reg.x = self.reg.x.wrapping_add(1);
        self.update_zero_and_negative_flags(self.reg.x);
        Ok(2)
    }
    
    /// INY - Increment Y Register
    fn iny(&mut self) -> CpuResult<u8> {
        self.reg.y = self.reg.y.wrapping_add(1);
        self.update_zero_and_negative_flags(self.reg.y);
        Ok(2)
    }
    
    /// DEC - Decrement Memory
    fn dec(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _, _) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr)?.wrapping_sub(1);
        memory.write_byte(addr, value)?;
        self.update_zero_and_negative_flags(value);
        
        Ok(match mode {
            AddressingMode::ZeroPage => 5,
            AddressingMode::ZeroPageX => 6,
            AddressingMode::Absolute => 6,
            AddressingMode::AbsoluteX => 7,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    /// DEX - Decrement X Register
    fn dex(&mut self) -> CpuResult<u8> {
        self.reg.x = self.reg.x.wrapping_sub(1);
        self.update_zero_and_negative_flags(self.reg.x);
        Ok(2)
    }
    
    /// DEY - Decrement Y Register
    fn dey(&mut self) -> CpuResult<u8> {
        self.reg.y = self.reg.y.wrapping_sub(1);
        self.update_zero_and_negative_flags(self.reg.y);
        Ok(2)
    }
    
    // =============================================
    // Shift/Rotate Operations
    // =============================================
    
    /// ASL - Arithmetic Shift Left (Accumulator)
    fn asl_acc(&mut self) -> CpuResult<u8> {
        // Set carry to bit 7 of accumulator
        self.reg.p.set(StatusFlags::CARRY, (self.reg.a & 0x80) != 0);
        
        // Shift left
        self.reg.a = self.reg.a.wrapping_shl(1);
        
        // Update flags
        self.update_zero_and_negative_flags(self.reg.a);
        
        Ok(2)
    }
    
    /// ASL - Arithmetic Shift Left (Memory)
    fn asl(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _, _) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr)?;
        
        // Set carry to bit 7 of value
        self.reg.p.set(StatusFlags::CARRY, (value & 0x80) != 0);
        
        // Shift left and write back
        let result = value.wrapping_shl(1);
        memory.write_byte(addr, result)?;
        
        // Update flags
        self.update_zero_and_negative_flags(result);
        
        Ok(match mode {
            AddressingMode::ZeroPage => 5,
            AddressingMode::ZeroPageX => 6,
            AddressingMode::Absolute => 6,
            AddressingMode::AbsoluteX => 7,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    /// LSR - Logical Shift Right (Accumulator)
    fn lsr_acc(&mut self) -> CpuResult<u8> {
        // Set carry to bit 0 of accumulator
        self.reg.p.set(StatusFlags::CARRY, (self.reg.a & 0x01) != 0);
        
        // Shift right
        self.reg.a = self.reg.a.wrapping_shr(1);
        
        // Update flags (bit 7 is always 0 after LSR)
        self.reg.p.set(StatusFlags::NEGATIVE, false);
        self.reg.p.set(StatusFlags::ZERO, self.reg.a == 0);
        
        Ok(2)
    }
    
    /// LSR - Logical Shift Right (Memory)
    fn lsr(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _, _) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr)?;
        
        // Set carry to bit 0 of value
        self.reg.p.set(StatusFlags::CARRY, (value & 0x01) != 0);
        
        // Shift right and write back
        let result = value.wrapping_shr(1);
        memory.write_byte(addr, result)?;
        
        // Update flags (bit 7 is always 0 after LSR)
        self.reg.p.set(StatusFlags::NEGATIVE, false);
        self.reg.p.set(StatusFlags::ZERO, result == 0);
        
        Ok(match mode {
            AddressingMode::ZeroPage => 5,
            AddressingMode::ZeroPageX => 6,
            AddressingMode::Absolute => 6,
            AddressingMode::AbsoluteX => 7,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    /// ROL - Rotate Left (Accumulator)
    fn rol_acc(&mut self) -> CpuResult<u8> {
        let old_carry = self.reg.p.contains(StatusFlags::CARRY);
        
        // Set carry to bit 7 of accumulator
        self.reg.p.set(StatusFlags::CARRY, (self.reg.a & 0x80) != 0);
        
        // Rotate left through carry
        self.reg.a = self.reg.a.wrapping_shl(1);
        if old_carry {
            self.reg.a |= 0x01;
        }
        
        // Update flags
        self.update_zero_and_negative_flags(self.reg.a);
        
        Ok(2)
    }
    
    /// ROL - Rotate Left (Memory)
    fn rol(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _, _) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr)?;
        let old_carry = self.reg.p.contains(StatusFlags::CARRY);
        
        // Set carry to bit 7 of value
        self.reg.p.set(StatusFlags::CARRY, (value & 0x80) != 0);
        
        // Rotate left through carry and write back
        let mut result = value.wrapping_shl(1);
        if old_carry {
            result |= 0x01;
        }
        memory.write_byte(addr, result)?;
        
        // Update flags
        self.update_zero_and_negative_flags(result);
        
        Ok(match mode {
            AddressingMode::ZeroPage => 5,
            AddressingMode::ZeroPageX => 6,
            AddressingMode::Absolute => 6,
            AddressingMode::AbsoluteX => 7,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    /// ROR - Rotate Right (Accumulator)
    fn ror_acc(&mut self) -> CpuResult<u8> {
        let old_carry = self.reg.p.contains(StatusFlags::CARRY);
        
        // Set carry to bit 0 of accumulator
        self.reg.p.set(StatusFlags::CARRY, (self.reg.a & 0x01) != 0);
        
        // Rotate right through carry
        self.reg.a = self.reg.a.wrapping_shr(1);
        if old_carry {
            self.reg.a |= 0x80;
        }
        
        // Update flags
        self.update_zero_and_negative_flags(self.reg.a);
        
        Ok(2)
    }
    
    /// ROR - Rotate Right (Memory)
    fn ror(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _, _) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr)?;
        let old_carry = self.reg.p.contains(StatusFlags::CARRY);
        
        // Set carry to bit 0 of value
        self.reg.p.set(StatusFlags::CARRY, (value & 0x01) != 0);
        
        // Rotate right through carry and write back
        let mut result = value.wrapping_shr(1);
        if old_carry {
            result |= 0x80;
        }
        memory.write_byte(addr, result)?;
        
        // Update flags
        self.update_zero_and_negative_flags(result);
        
        Ok(match mode {
            AddressingMode::ZeroPage => 5,
            AddressingMode::ZeroPageX => 6,
            AddressingMode::Absolute => 6,
            AddressingMode::AbsoluteX => 7,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    /// LDX - Load X Register
    fn ldx(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = if matches!(mode, AddressingMode::Immediate) {
            addr as u8
        } else {
            memory.read_byte(addr)?
        };
        
        self.reg.x = value;
        self.update_zero_and_negative_flags(self.reg.x);
        
        // Base cycles + extra cycles for page crossing
        Ok(match mode {
            AddressingMode::Immediate => 2,
            AddressingMode::ZeroPage => 3,
            AddressingMode::ZeroPageY => 4,
            AddressingMode::Absolute => 4,
            AddressingMode::AbsoluteY => 4 + extra_cycles,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    /// LDY - Load Y Register
    fn ldy(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = if matches!(mode, AddressingMode::Immediate) {
            addr as u8
        } else {
            memory.read_byte(addr)?
        };
        
        self.reg.y = value;
        self.update_zero_and_negative_flags(self.reg.y);
        
        // Base cycles + extra cycles for page crossing
        Ok(match mode {
            AddressingMode::Immediate => 2,
            AddressingMode::ZeroPage => 3,
            AddressingMode::ZeroPageX => 4,
            AddressingMode::Absolute => 4,
            AddressingMode::AbsoluteX => 4 + extra_cycles,

// Implement Memory trait for Cpu to allow direct memory access
impl<M: Memory> Memory for Cpu<M> {
    fn read_byte(&self, addr: u16) -> Result<u8, MemoryError> {
        self.memory.read_byte(addr)
    }

    fn write_byte(&mut self, addr: u16, value: u8) -> Result<(), MemoryError> {
        self.memory.write_byte(addr, value)
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
