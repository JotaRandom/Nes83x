use super::{Cpu, CpuResult, CpuError, AddressingMode, StatusFlags};
use crate::nes::utils::Memory;

impl Cpu {
    /// AXS (SBX) - AND X register with accumulator and store in X
    /// Then subtract operand from X without borrow (CMP & DEX at once)
    pub(crate) fn axs(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _page_crossed, _) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr).map_err(CpuError::MemoryError)?;
        
        // AND X with A and store in X
        let x = self.reg.x & self.reg.a;
        self.reg.x = x;
        
        // Compare with value from memory
        let result = x.wrapping_sub(value);
        
        // Update flags
        self.reg.p.set(StatusFlags::CARRY, x >= value);
        self.update_zero_and_negative_flags(result);
        
        // Set X to the result
        self.reg.x = result;
        
        // Return cycle count based on addressing mode
        match mode {
            AddressingMode::Immediate => Ok(2),
            AddressingMode::ZeroPage => Ok(3),
            AddressingMode::Absolute => Ok(4),
            _ => Err(CpuError::InvalidAddressingMode(mode)),
        }
    }
    
    /// DOP (Double NOP) - No operation (2-byte NOP)
    pub(crate) fn dop(&mut self, _mode: AddressingMode, _memory: &mut impl Memory) -> CpuResult<u8> {
        // Just skip the immediate byte
        self.reg.pc = self.reg.pc.wrapping_add(1);
        Ok(2)
    }
    
    /// STP (STOP) - Stop the processor (HALT)
    pub(crate) fn stp(&mut self, _mode: AddressingMode, _memory: &mut impl Memory) -> CpuResult<u8> {
        // In a real 6502, this would halt the CPU until a reset
        // For emulation purposes, we'll just stop execution
        self.is_running = false;
        Ok(0)
    }
    
    /// LAS (LAR) - AND memory with stack pointer, transfer to A, X, and S
    pub(crate) fn las(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, _) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr).map_err(CpuError::MemoryError)?;
        
        // AND memory with stack pointer, store in A, X, and S
        let result = value & self.reg.s;
        self.reg.a = result;
        self.reg.x = result;
        self.reg.s = result;
        
        // Update flags
        self.update_zero_and_negative_flags(result);
        
        // Return cycle count based on addressing mode
        match mode {
            AddressingMode::AbsoluteY => Ok(4 + if page_crossed { 1 } else { 0 }),
            _ => Ok(4),
        }
    }
}
