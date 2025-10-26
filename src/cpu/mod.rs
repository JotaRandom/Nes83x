//! 6502 CPU emulation for the NES

use bitflags::bitflags;
use thiserror::Error;
use std::fmt;
use std::io;

/// Represents the result of a memory access operation
pub type CpuResult<T> = Result<T, CpuError>;

bitflags! {
    /// CPU Status Register (P) flags
    #[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct StatusFlags: u8 {
        /// Carry Flag
        const CARRY = 0b0000_0001;
        /// Zero Flag
        const ZERO = 0b0000_0010;
        /// Interrupt Disable
        const INTERRUPT_DISABLE = 0b0000_0100;
        /// Decimal Mode (not used in NES, but present on 6502)
        const DECIMAL = 0b0000_1000;
        /// Break Command
        const BREAK = 0b0001_0000;
        /// Unused (always 1)
        const UNUSED = 0b0010_0000;
        /// Overflow Flag
        const OVERFLOW = 0b0100_0000;
        /// Negative Flag
        const NEGATIVE = 0b1000_0000;
    }
}

/// Represents the 6502 CPU registers
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

/// Represents the 6502 CPU
pub struct Cpu {
    /// CPU registers
    pub reg: Registers,
    /// Number of cycles remaining for current instruction
    pub cycles: u32,
    /// Whether an NMI is pending
    pub nmi_pending: bool,
    /// Whether an IRQ is pending
    pub irq_pending: bool,
    /// Whether the CPU is in a valid state
    pub is_running: bool,
}

/// Errors that can occur during CPU operations
#[derive(Error, Debug)]
pub enum CpuError {
    #[error("Invalid opcode: {0:02X}")]
    InvalidOpcode(u8),
    
    #[error("CPU is in an invalid state")]
    InvalidState,
    
    #[error("Memory access error: {0}")]
    MemoryError(#[from] std::io::Error),
    
    #[error("Invalid addressing mode: {0:?}")]
    InvalidAddressingMode(AddressingMode),
    
    #[error("Invalid memory access at address: {0:04X}")]
    InvalidMemoryAccess(u16),
    
    #[error("Stack overflow")]
    StackOverflow,
    
    #[error("Stack underflow")]
    StackUnderflow,
}

impl Cpu {
    /// Create a new CPU instance
    pub fn new() -> Self {
        Cpu {
            reg: Registers {
                a: 0,
                x: 0,
                y: 0,
                s: 0xFD, // Default stack pointer
                pc: 0,
                p: StatusFlags::UNUSED | StatusFlags::INTERRUPT_DISABLE,
            },
            cycles: 0,
            nmi_pending: false,
            irq_pending: false,
            is_running: false,
        }
    }
    
    /// Reset the CPU to its initial state
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
    
    /// Reset the CPU to its initial state
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
    
    /// Trigger a non-maskable interrupt (NMI)
    pub fn trigger_nmi(&mut self) {
        self.nmi_pending = true;
    }
    
    /// Trigger an interrupt request (IRQ)
    pub fn trigger_irq(&mut self) {
        if !self.reg.p.contains(StatusFlags::INTERRUPT_DISABLE) {
            self.irq_pending = true;
        }
    }
    
    /// Execute a single CPU instruction
    pub fn step(&mut self, memory: &mut impl Memory) -> Result<u32, CpuError> {
        if !self.is_running {
            return Err(CpuError::InvalidState);
        }
        
        // Handle pending interrupts
        if self.nmi_pending {
            self.handle_interrupt(memory, 0xFFFA)?;
            self.nmi_pending = false;
            self.cycles += 7;
            return Ok(7);
        } else if self.irq_pending {
            self.handle_interrupt(memory, 0xFFFE)?;
            self.irq_pending = false;
            self.cycles += 7;
            return Ok(7);
        }
        
        // Fetch and execute instruction
        let opcode = self.read_byte(memory, self.reg.pc)?;
        self.reg.pc = self.reg.pc.wrapping_add(1);
        
        // Execute the instruction (stub implementation)
        let cycles = self.execute_instruction(opcode, memory)?;
        self.cycles += cycles as u32;
        
        Ok(cycles as u32)
    }
    
    /// Execute a single CPU instruction (internal implementation)
    fn execute_instruction(&mut self, opcode: u8, memory: &mut impl Memory) -> CpuResult<u8> {
        match opcode {
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
            0x11 => self.ora(AddressingMode::IndirectY, memory),    // ORA (Indirect),Y
            
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
            0x4C => self.jmp_absolute(memory),                      // JMP Absolute
            0x6C => self.jmp_indirect(memory),                      // JMP Indirect
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
            0x0B | 0x2B => self.aac(opcode, memory),                                  // AAC (ANC)
            0x8B => self.ane(memory),                                                 // ANE (XAA)
            0x4B => self.asr(memory),                                                 // ASR (ALR)
            0x6B => self.arr(memory),                                                 // ARR
            0xCB => self.axs(memory),                                                 // AXS (SBX)
            0xE2 => self.dop(memory),                                                 // DOP (NOP)
            0x02 | 0x12 | 0x22 | 0x32 | 0x42 | 0x52 | 0x62 | 0x72 | 0x92 | 0xB2 | 0xD2 | 0xF2 => 
                self.stp(),                                                          // STP (KIL)
            0xBB => self.las(memory),                                                 // LAS (LAX)
            0xA7 => self.lax(AddressingMode::ZeroPage, memory),                      // LAX Zero Page
            0xB7 => self.lax(AddressingMode::ZeroPageY, memory),                     // LAX Zero Page,Y
            0xAF => self.lax(AddressingMode::Absolute, memory),                      // LAX Absolute
            0xBF => self.lax(AddressingMode::AbsoluteY, memory),                     // LAX Absolute,Y
            0xA3 => self.lax(AddressingMode::IndirectX, memory),                     // LAX (Indirect,X)
            0xB3 => self.lax(AddressingMode::IndirectY, memory),                     // LAX (Indirect),Y
            0x80 | 0x82 | 0x89 | 0xC2 | 0xE2 => self.nop(memory),                    // NOP (skips a byte)
            0x04 | 0x44 | 0x64 => self.nop(memory),                                  // NOP Zero Page
            0x14 | 0x34 | 0x54 | 0x74 | 0xD4 | 0xF4 => self.nop(memory),            // NOP Zero Page,X
            0x0C => self.nop(memory),                                                // NOP Absolute
            0x1C | 0x3C | 0x5C | 0x7C | 0xDC | 0xFC => self.nop(memory),            // NOP Absolute,X
            0x1A | 0x3A | 0x5A | 0x7A | 0xDA | 0xFA => self.nop(memory),            // NOP Implied
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
            0x87 => self.sax(AddressingMode::ZeroPage, memory),                      // SAX Zero Page
            0x97 => self.sax(AddressingMode::ZeroPageY, memory),                     // SAX Zero Page,Y
            0x8F => self.sax(AddressingMode::Absolute, memory),                      // SAX Absolute
            0x83 => self.sax(AddressingMode::IndirectX, memory),                     // SAX (Indirect,X)
            0xEB => self.sbc(AddressingMode::Immediate, memory),                     // SBC (SBC with same opcode as 0xE9)
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
            0x9E => self.sha(AddressingMode::AbsoluteY, memory),                     // SHA (AXS) Absolute,Y
            0x9F => self.sha(AddressingMode::AbsoluteY, memory),                     // SHA (AXS) Absolute,Y (alternate)
            0x93 => self.sha(AddressingMode::IndirectY, memory),                     // SHA (AXS) (Indirect),Y
            0x9C => self.shy(AddressingMode::AbsoluteX, memory),                     // SHY Absolute,X
            0x9B => self.shx(AddressingMode::AbsoluteY, memory),                     // SHX Absolute,Y
            0x9F => self.sxa(AddressingMode::AbsoluteY, memory),                     // SXA (SHX) Absolute,Y
            0x9E => self.sya(AddressingMode::AbsoluteX, memory),                     // SYA (SHY) Absolute,X
            0x8F => self.xas(AddressingMode::AbsoluteY, memory),                     // XAS (SHX) Absolute,Y
            0x9E => self.xas(AddressingMode::AbsoluteX, memory),                     // XAS (SHY) Absolute,X
            0x9C => self.xas(AddressingMode::AbsoluteX, memory),                     // XAS (SHX) Absolute,X (alternate)
            0x9B => self.xas(AddressingMode::AbsoluteY, memory),                     // XAS (SHY) Absolute,Y (alternate)
            0x9F => self.xas(AddressingMode::AbsoluteY, memory),                     // XAS (SHX) Absolute,Y (alternate)
            0x93 => self.xas(AddressingMode::IndirectY, memory),                     // XAS (SHX) (Indirect),Y
            
            // Handle unknown opcodes
            _ => {
                log::warn!("Unknown opcode: {:#04X} at PC: {:#06X}", opcode, self.reg.pc);
                // Treat as a 1-byte NOP for compatibility
                Ok(1)
            }
    }
    
    /// Handle an interrupt
    fn handle_interrupt(&mut self, memory: &mut impl Memory, vector: u16) -> Result<(), CpuError> {
        // Push PC and status register to stack
        self.push_word(memory, self.reg.pc)?;
        self.push_byte(memory, self.reg.p.bits() | StatusFlags::BREAK.bits() | StatusFlags::UNUSED.bits())?;
        
        // Set interrupt disable flag
        self.reg.p.insert(StatusFlags::INTERRUPT_DISABLE);
        
        // Set PC to interrupt vector
        self.reg.pc = memory.read_word(vector)?;
        
        Ok(())
    }
    
    // =============================================
    // Addressing Modes
    // =============================================
    
    /// Get the address for the specified addressing mode
    /// Returns (address, page_crossed, cycles)
    fn get_operand_address(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<(u16, bool, u8)> {
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
                let addr = base.wrapping_add(self.reg.x as u16) & 0x00FF;
                Ok((addr, false, 0))
            },
            AddressingMode::ZeroPageY => {
                let base = memory.read_byte(self.reg.pc)? as u16;
                self.reg.pc = self.reg.pc.wrapping_add(1);
                let addr = base.wrapping_add(self.reg.y as u16) & 0x00FF;
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
                let page_crossed = (base & 0xFF00) != (addr & 0xFF00);
                Ok((addr, page_crossed, if page_crossed { 1 } else { 0 }))
            },
            AddressingMode::AbsoluteY => {
                let base = memory.read_word(self.reg.pc)?;
                self.reg.pc = self.reg.pc.wrapping_add(2);
                let addr = base.wrapping_add(self.reg.y as u16);
                let page_crossed = (base & 0xFF00) != (addr & 0xFF00);
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
                let base = memory.read_byte(self.reg.pc)? as u8;
                self.reg.pc = self.reg.pc.wrapping_add(1);
                let ptr = base.wrapping_add(self.reg.x);
                let lo = memory.read_byte(ptr as u16)? as u16;
                let hi = memory.read_byte(ptr.wrapping_add(1) as u16)? as u16;
                Ok(((hi << 8) | lo, false, 0))
            },
            AddressingMode::IndirectY => {
                let base = memory.read_byte(self.reg.pc)? as u8;
                self.reg.pc = self.reg.pc.wrapping_add(1);
                let lo = memory.read_byte(base as u16)? as u16;
                let hi = memory.read_byte(base.wrapping_add(1) as u16)? as u16;
                let deref_base = (hi << 8) | lo;
                let addr = deref_base.wrapping_add(self.reg.y as u16);
                let page_crossed = (deref_base & 0xFF00) != (addr & 0xFF00);
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
    // Stack Operations
    // =============================================
    
    fn push_byte(&mut self, memory: &mut impl Memory, value: u8) -> CpuResult<()> {
        memory.write_byte(0x0100 | (self.reg.s as u16), value)?;
        self.reg.s = self.reg.s.wrapping_sub(1);
        Ok(())
    }
    
    fn push_word(&mut self, memory: &mut impl Memory, value: u16) -> CpuResult<()> {
        let [hi, lo] = value.to_be_bytes();
        self.push_byte(memory, hi)?;
        self.push_byte(memory, lo)
    }
    
    fn pull_byte(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        self.reg.s = self.reg.s.wrapping_add(1);
        memory.read_byte(0x0100 | (self.reg.s as u16)).map_err(Into::into)
    }
    
    fn pull_word(&mut self, memory: &mut impl Memory) -> CpuResult<u16> {
        let lo = self.pull_byte(memory)? as u16;
        let hi = self.pull_byte(memory)? as u16;
        Ok((hi << 8) | lo)
    }
    
    // =============================================
    // Flag Operations
    // =============================================
    
    fn update_zero_and_negative_flags(&mut self, result: u8) {
        self.reg.p.set(StatusFlags::ZERO, result == 0);
        self.reg.p.set(StatusFlags::NEGATIVE, (result & 0x80) != 0);
    }
    
    fn set_overflow_flag(&mut self, a: u8, b: u8, result: u8) {
        // Overflow occurs if the sign of both inputs is the same and different from the result
        let overflow = ((a ^ result) & (b ^ result) & 0x80) != 0;
        self.reg.p.set(StatusFlags::OVERFLOW, overflow);
    }
    
    // =============================================
    // Arithmetic and Logic Operations
    // =============================================
    
    /// ADC - Add with Carry
    fn adc(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = if matches!(mode, AddressingMode::Immediate) {
            addr as u8
        } else {
            memory.read_byte(addr)?
        };
        
        let carry = if self.reg.p.contains(StatusFlags::CARRY) { 1 } else { 0 };
        let sum = self.reg.a as u16 + value as u16 + carry;
        
        // Set carry flag if sum > 255
        self.reg.p.set(StatusFlags::CARRY, sum > 0xFF);
        
        // Set overflow flag if sign bit is incorrect
        let result = sum as u8;
        self.set_overflow_flag(self.reg.a, value, result);
        
        self.reg.a = result;
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
    
    /// SBC - Subtract with Carry
    fn sbc(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        // SBC is the same as ADC with the operand's bits inverted
        // and the carry flag treated as inverted
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
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
        
        // Set overflow flag if sign bit is incorrect
        let result = sum as u8;
        self.set_overflow_flag(self.reg.a, !value, result);
        
        self.reg.a = result;
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Same cycle counts as ADC
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
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = if matches!(mode, AddressingMode::Immediate) {
            addr as u8
        } else {
            memory.read_byte(addr)?
        };
        
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
    
    // =============================================
    // Load/Store Operations
    // =============================================
    
    /// LDA - Load Accumulator
    fn lda(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = if matches!(mode, AddressingMode::Immediate) {
            addr as u8
        } else {
            memory.read_byte(addr)?
        };
        
        self.reg.a = value;
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Base cycles + extra cycles for page crossing
        Ok(match mode {
            AddressingMode::Immediate => 2,
            AddressingMode::ZeroPage => 3,
            AddressingMode::ZeroPageX | AddressingMode::ZeroPageY => 4,
            AddressingMode::Absolute => 4,
            AddressingMode::AbsoluteX | AddressingMode::AbsoluteY => 4 + extra_cycles,
            AddressingMode::IndirectX => 6,
            AddressingMode::IndirectY => 5 + extra_cycles,
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
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    /// STA - Store Accumulator
    fn sta(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _, _) = self.get_operand_address(mode, memory)?;
        memory.write_byte(addr, self.reg.a)?;
        
        Ok(match mode {
            AddressingMode::ZeroPage => 3,
            AddressingMode::ZeroPageX => 4,
            AddressingMode::Absolute => 4,
            AddressingMode::AbsoluteX | AddressingMode::AbsoluteY => 5,
            AddressingMode::IndirectX => 6,
            AddressingMode::IndirectY => 6,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    /// STX - Store X Register
    fn stx(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _, _) = self.get_operand_address(mode, memory)?;
        memory.write_byte(addr, self.reg.x)?;
        
        Ok(match mode {
            AddressingMode::ZeroPage => 3,
            AddressingMode::ZeroPageY => 4,
            AddressingMode::Absolute => 4,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    /// STY - Store Y Register
    fn sty(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _, _) = self.get_operand_address(mode, memory)?;
        memory.write_byte(addr, self.reg.y)?;
        
        Ok(match mode {
            AddressingMode::ZeroPage => 3,
            AddressingMode::ZeroPageX => 4,
            AddressingMode::Absolute => 4,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        })
    }
    
    // =============================================
    // Register Transfers
    // =============================================
    
    /// TAX - Transfer Accumulator to X
    fn tax(&mut self) -> CpuResult<u8> {
        self.reg.x = self.reg.a;
        self.update_zero_and_negative_flags(self.reg.x);
        Ok(2)
    }
    
    /// TXA - Transfer X to Accumulator
    fn txa(&mut self) -> CpuResult<u8> {
        self.reg.a = self.reg.x;
        self.update_zero_and_negative_flags(self.reg.a);
        Ok(2)
    }
    
    /// TAY - Transfer Accumulator to Y
    fn tay(&mut self) -> CpuResult<u8> {
        self.reg.y = self.reg.a;
        self.update_zero_and_negative_flags(self.reg.y);
        Ok(2)
    }
    
    /// TYA - Transfer Y to Accumulator
    fn tya(&mut self) -> CpuResult<u8> {
        self.reg.a = self.reg.y;
        self.update_zero_and_negative_flags(self.reg.a);
        Ok(2)
    }
    
    /// TSX - Transfer Stack Pointer to X
    fn tsx(&mut self) -> CpuResult<u8> {
        self.reg.x = self.reg.s;
        self.update_zero_and_negative_flags(self.reg.x);
        Ok(2)
    }
    
    /// TXS - Transfer X to Stack Pointer
    fn txs(&mut self) -> CpuResult<u8> {
        self.reg.s = self.reg.x;
        // Note: Does not affect flags
        Ok(2)
    }
    
    // =============================================
    // Stack Operations
    // =============================================
    
    /// PHA - Push Accumulator
    fn pha(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        self.push_byte(memory, self.reg.a)?;
        Ok(3)
    }
    
    /// PHP - Push Processor Status
    fn php(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        // Set Break and Unused flags when pushing
        let status = self.reg.p | StatusFlags::BREAK | StatusFlags::UNUSED;
        self.push_byte(memory, status.bits())?;
        Ok(3)
    }
    
    /// PLA - Pull Accumulator
    fn pla(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        self.reg.a = self.pull_byte(memory)?;
        self.update_zero_and_negative_flags(self.reg.a);
        Ok(4)
    }
    
    /// PLP - Pull Processor Status
    fn plp(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        let status = self.pull_byte(memory)?;
        // Clear Break and Unused flags when pulling
        self.reg.p = StatusFlags::from_bits_truncate(status) & !(StatusFlags::BREAK | StatusFlags::UNUSED);
        // Ensure Unused flag is always set
        self.reg.p.insert(StatusFlags::UNUSED);
        Ok(4)
    }
        
        // Jump to interrupt vector
        self.reg.pc = self.read_word(memory, vector)?;
        
        Ok(())
    }
    
    // Helper methods for memory access
    fn read_byte(&self, memory: &impl Memory, addr: u16) -> Result<u8, CpuError> {
        memory.read_byte(addr).map_err(CpuError::MemoryError)
    }
    
    fn write_byte(&self, memory: &mut impl Memory, addr: u16, value: u8) -> Result<(), CpuError> {
        memory.write_byte(addr, value).map_err(CpuError::MemoryError)
    }
    
    fn read_word(&self, memory: &impl Memory, addr: u16) -> Result<u16, Result<u16, CpuError>> {
        let lo = self.read_byte(memory, addr).map_err(Err)?;
        let hi = self.read_byte(memory, addr.wrapping_add(1)).map_err(Err)?;
        Ok(u16::from_le_bytes([lo, hi]))
    }
    
    fn push_byte(&mut self, memory: &mut impl Memory, value: u8) -> Result<(), CpuError> {
        let addr = 0x0100 | (self.reg.s as u16);
        self.write_byte(memory, addr, value)?;
        self.reg.s = self.reg.s.wrapping_sub(1);
        Ok(())
    }
    
    fn push_word(&mut self, memory: &mut impl Memory, value: u16) -> Result<(), CpuError> {
        let [hi, lo] = value.to_be_bytes();
        self.push_byte(memory, hi)?;
        self.push_byte(memory, lo)?;
        Ok(())
    }
    
    fn pop_byte(&mut self, memory: &mut impl Memory) -> Result<u8, CpuError> {
        self.reg.s = self.reg.s.wrapping_add(1);
        let addr = 0x0100 | (self.reg.s as u16);
        self.read_byte(memory, addr)
    }
    
    fn pop_word(&mut self, memory: &mut impl Memory) -> Result<u16, CpuError> {
        let lo = self.pop_byte(memory)? as u16;
        let hi = self.pop_byte(memory)? as u16;
        Ok((hi << 8) | lo)
    }
    
    // Helper methods for flag operations
    /// Execute a branch instruction
    fn branch(&mut self, condition: bool, memory: &mut impl Memory) -> CpuResult<u8> {
        if !condition {
            // If branch not taken, just skip the offset byte
            self.reg.pc = self.reg.pc.wrapping_add(1);
            return Ok(2);  // 2 cycles if branch not taken
        }
        
        // Read the offset (signed 8-bit value)
        let offset = self.read_byte(memory, self.reg.pc)? as i8;
        let old_pc = self.reg.pc;
        
        // Calculate new PC (takes 1 extra cycle if branch is taken)
        self.reg.pc = self.reg.pc.wrapping_add(1);  // Skip the offset byte
        let new_pc = self.reg.pc.wrapping_add(offset as u16);
        self.reg.pc = new_pc;
        
        // Check for page crossing (adds 1 more cycle if page boundary is crossed)
        let page_crossed = (old_pc & 0xFF00) != (new_pc & 0xFF00);
        
        Ok(3 + if page_crossed { 1 } else { 0 })
    }
    
    // =============================================
    // Jump & Subroutine Operations
    // =============================================
    
    /// JMP - Jump to new location (Absolute)
    fn jmp_absolute(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        let addr = memory.read_word(self.reg.pc)?;
        self.reg.pc = addr;
        Ok(3)
    }
    
    /// JMP - Jump to new location (Indirect)
    fn jmp_indirect(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        let ptr = memory.read_word(self.reg.pc)?;
        
        // Handle 6502 page boundary bug
        let lo = memory.read_byte(ptr)? as u16;
        let hi_ptr = if (ptr & 0x00FF) == 0x00FF {
            ptr & 0xFF00  // Wrap around within page
        } else {
            ptr.wrapping_add(1)
        };
        let hi = memory.read_byte(hi_ptr)? as u16;
        
        self.reg.pc = (hi << 8) | lo;
        Ok(5)
    }
    
    /// JSR - Jump to Subroutine
    fn jsr(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        let return_addr = self.reg.pc.wrapping_add(2);  // Address after the JSR instruction
        self.push_word(memory, return_addr - 1)?;  // Push return address - 1 (6502 quirk)
        
        // Set PC to the target address
        let addr = memory.read_word(self.reg.pc)?;
        self.reg.pc = addr;
        
        Ok(6)
    }
    
    /// RTS - Return from Subroutine
    fn rts(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        let return_addr = self.pop_word(memory)?;
        self.reg.pc = return_addr.wrapping_add(1);  // Add 1 to get the correct return address
        Ok(6)
    }
    
    /// RTI - Return from Interrupt
    fn rti(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        // Pull processor status
        let status = self.pop_byte(memory)?;
        self.reg.p = StatusFlags::from_bits_truncate(status);
        
        // Pull program counter
        let pc_lo = self.pop_byte(memory)? as u16;
        let pc_hi = self.pop_byte(memory)? as u16;
        self.reg.pc = (pc_hi << 8) | pc_lo;
        
        // Clear the BREAK flag in the status register
        self.reg.p.remove(StatusFlags::BREAK);
        
        Ok(6)
    }
    
    fn update_zero_and_negative_flags(&mut self, result: u8) {
        self.reg.p.set(StatusFlags::ZERO, result == 0);
        self.reg.p.set(StatusFlags::NEGATIVE, (result & 0x80) != 0);
    }
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

/// Trait for memory access
pub trait Memory {
    /// Read a byte from memory
    fn read_byte(&self, addr: u16) -> io::Result<u8>;
    
    /// Write a byte to memory
    fn write_byte(&mut self, addr: u16, value: u8) -> io::Result<()>;
    
    /// Read a word (little-endian) from memory
    fn read_word(&self, addr: u16) -> io::Result<u16> {
        let lo = self.read_byte(addr)? as u16;
        let hi = self.read_byte(addr.wrapping_add(1))? as u16;
        Ok((hi << 8) | lo)
    }
    
    /// Write a word (little-endian) to memory
    fn write_word(&mut self, addr: u16, value: u16) -> io::Result<()> {
        let [lo, hi] = value.to_le_bytes();
        self.write_byte(addr, lo)?;
        self.write_byte(addr.wrapping_add(1), hi)
    }
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
