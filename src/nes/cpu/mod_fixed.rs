//! # 6502 CPU Emulation for the NES
//! 
//! This module implements a cycle-accurate emulation of the Ricoh 2A03/2A07 CPU,
//! which is a variant of the MOS Technology 6502 microprocessor used in the NES.

use super::utils::Memory;
use super::registers::Registers;
use std::io;

// Re-export the Memory trait
pub use crate::nes::utils::Memory;

mod status_flags;
mod unofficial_ops;
#[cfg(feature = "unofficial_ops")]
mod unofficial_ops_consolidated;

pub use self::status_flags::StatusFlags;

/// Represents the result of a memory access operation
pub type CpuResult<T> = Result<T, CpuError>;

/// # 6502 CPU Registers
///
/// The 6502 has three general-purpose 8-bit registers (A, X, Y),
/// a program counter (PC), stack pointer (S), and a status register (P).
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
    pub memory: M,
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
    pub fn new() -> Self {
        use crate::nes::utils::MemoryBlock;
        
        // Create a default memory block (can be replaced with actual memory later)
        let default_memory = vec![0u8; 0x10000]; // 64KB of memory
        let memory = Box::new(MemoryBlock::new(default_memory.leak()));
        
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
        self.irq_pending = true;
    }
    
    /// Executes a single CPU instruction
    pub fn step(&mut self, memory: &mut M) -> Result<u32, CpuError> {
        if !self.is_running {
            return Err(CpuError::InvalidState);
        }
        
        // Handle pending interrupts
        if self.nmi_pending {
            self.nmi_pending = false;
            let cycles = self.handle_interrupt(memory, 0xFFFA, false)?;
            return Ok(cycles as u32);
        }
        
        if !self.reg.p.contains(StatusFlags::INTERRUPT_DISABLE) && self.irq_pending {
            self.irq_pending = false;
            let cycles = self.handle_interrupt(memory, 0xFFFE, false)?;
            return Ok(cycles as u32);
        }
        
        // Fetch and execute the next instruction
        let opcode = memory.read_byte(self.reg.pc)?;
        self.reg.pc = self.reg.pc.wrapping_add(1);
        
        let cycles = self.execute_instruction(opcode, memory)?;
        Ok(cycles as u32)
    }
    
    /// Execute a single CPU instruction (internal implementation)
    fn execute_instruction<M2: Memory>(&mut self, opcode: u8, memory: &mut M2) -> CpuResult<u8> {
        match opcode {
            // Unofficial opcodes (enabled with the 'unofficial_ops' feature)
            #[cfg(feature = "unofficial_ops")]
            0x0B | 0x2B => self.aac(memory),  // AAC (ANC)
            #[cfg(feature = "unofficial_ops")]
            0x8B => self.ane(memory),         // ANE (XAA)
            #[cfg(feature = "unofficial_ops")]
            0x6B => self.arr(memory),         // ARR
            
            // Add all other opcode patterns here...
            // This is a simplified version - you'll need to add all the opcodes
            
            _ => Err(CpuError::InvalidOpcode(opcode)),
        }
    }
    
    /// Handle an interrupt request (NMI, IRQ, or BRK)
    fn handle_interrupt(&mut self, memory: &mut impl Memory, vector: u16, is_brk: bool) -> CpuResult<u8> {
        // Push PC and status register
        self.push_word(memory, self.reg.pc)?;
        
        // Set the B flag if this is a BRK instruction
        let mut status = self.reg.p;
        if is_brk {
            status.insert(StatusFlags::BREAK);
        }
        self.push_byte(memory, status.bits())?;
        
        // Set interrupt disable flag
        self.reg.p.insert(StatusFlags::INTERRUPT_DISABLE);
        
        // Jump to the interrupt vector
        self.reg.pc = memory.read_word(vector)?;
        
        // Return the number of cycles used (7 for NMI/IRQ, 7 for BRK)
        Ok(7)
    }
    
    // Add other helper methods here...
    
    /// Push a byte onto the stack
    fn push_byte(&mut self, memory: &mut impl Memory, value: u8) -> CpuResult<()> {
        let addr = 0x0100 | (self.reg.s as u16);
        memory.write_byte(addr, value)?;
        self.reg.s = self.reg.s.wrapping_sub(1);
        Ok(())
    }
    
    /// Push a word (16-bit value) onto the stack
    fn push_word(&mut self, memory: &mut impl Memory, value: u16) -> CpuResult<()> {
        self.push_byte(memory, (value >> 8) as u8)?;
        self.push_byte(memory, value as u8)
    }
    
    /// Pull a byte from the stack
    fn pull_byte(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        self.reg.s = self.reg.s.wrapping_add(1);
        let addr = 0x0100 | (self.reg.s as u16);
        memory.read_byte(addr)
    }
    
    /// Pull a word (16-bit value) from the stack
    fn pull_word(&mut self, memory: &mut impl Memory) -> CpuResult<u16> {
        let lo = self.pull_byte(memory)? as u16;
        let hi = self.pull_byte(memory)? as u16;
        Ok((hi << 8) | lo)
    }
    
    // Add more CPU instructions and helper methods here...
}

// Add tests and other implementations...

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nes::utils::test_utils::TestMemory;
    
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
