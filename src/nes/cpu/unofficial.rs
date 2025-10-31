//! Unofficial/illegal 6502 opcode implementations for the NES CPU.
//!
//! This module contains implementations of unofficial/illegal opcodes that were
//! used in some commercial NES games. These opcodes are not part of the
//! official 6502 instruction set but were discovered to have consistent
//! behavior across different 6502 variants.

use super::{Cpu, CpuResult, CpuError, AddressingMode, StatusFlags};
use crate::nes::utils::Memory;

// Include the additional implementations
// These are now directly in this file, so no need for separate modules

// Re-export the additional implementations from other files
pub use super::unofficial_impls::*;
pub use super::more_unofficial::*;

impl Cpu {
    /// AAC (ANC) - AND Accumulator with Memory then Move Bit 7 to Carry
    /// 
    /// This unofficial opcode performs a bitwise AND between the accumulator and a memory value,
    /// then moves bit 7 of the result to the carry flag. This opcode is used in some commercial
    /// games for copy protection and other purposes.
    /// 
    /// # Opcodes
    /// - 0x0B: ANC #imm (Immediate)
    /// - 0x2B: ANC #imm (Immediate, alternate encoding)
    /// 
    /// # Flags Affected
    /// - N: Set if bit 7 of the result is set
    /// - Z: Set if the result is zero
    /// - C: Set to the value of bit 7 of the result
    pub(crate) fn aac(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        let value = memory.read_byte(self.reg.pc).map_err(CpuError::MemoryError)?;
        self.reg.pc = self.reg.pc.wrapping_add(1);
        
        // Perform AND operation
        self.reg.a &= value;
        
        // Set Zero and Negative flags
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Set Carry flag to bit 7 of the result
        self.reg.p.set(StatusFlags::CARRY, (self.reg.a & 0x80) != 0);
        
        Ok(2)
    }
    
    /// ANE (XAA) - AND X with Memory then AND with High Byte of (Address+1)
    /// 
    /// This is an undocumented instruction that performs a complex operation used in some
    /// copy protection schemes. The exact behavior varies between different 6502 variants.
    /// 
    /// # Opcodes
    /// - 0x8B: ANE #imm (Immediate)
    /// 
    /// # Flags Affected
    /// - N: Set if bit 7 of the result is set
    /// - Z: Set if the result is zero
    pub(crate) fn ane(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        let value = memory.read_byte(self.reg.pc).map_err(CpuError::MemoryError)?;
        self.reg.pc = self.reg.pc.wrapping_add(1);
        
        // The exact operation is not well documented and varies between CPU revisions
        // This is a common implementation that works for most cases
        self.reg.a = self.reg.x & self.reg.a & value;
        
        // Update flags
        self.update_zero_and_negative_flags(self.reg.a);
        
        Ok(2)
    }
    
    /// ARR - AND Accumulator then Rotate Right
    /// 
    /// This is an undocumented instruction that performs an AND operation followed by
    /// a ROR operation, with special handling of the carry flag.
    /// 
    /// # Opcodes
    /// - 0x6B: ARR #imm (Immediate)
    /// 
    /// # Flags Affected
    /// - N: Set if bit 6 of the result is set (not bit 7 like normal!)
    /// - Z: Set if the result is zero
    /// - C: Set to the old value of bit 6 of the accumulator
    /// - V: Set to the XOR of bits 6 and 5 of the result
    pub(crate) fn arr(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        let value = memory.read_byte(self.reg.pc).map_err(CpuError::MemoryError)?;
        self.reg.pc = self.reg.pc.wrapping_add(1);
        
        // Perform AND operation
        self.reg.a &= value;
        
        // Save old carry for V flag calculation
        let old_carry = self.reg.p.contains(StatusFlags::CARRY);
        
        // Set carry flag to bit 7
        let new_carry = (self.reg.a & 0x80) != 0;
        
        // Rotate right through carry
        let mut result = self.reg.a >> 1;
        if old_carry {
            result |= 0x80;
        }
        
        self.reg.a = result;
        
        // Update flags
        self.reg.p.set(StatusFlags::CARRY, new_carry);
        
        // Set V flag to XOR of bits 6 and 5 of the result
        let bit6 = (result & 0x40) != 0;
        let bit5 = (result & 0x20) != 0;
        self.reg.p.set(StatusFlags::OVERFLOW, bit6 ^ bit5);
        
        // Set N flag based on bit 6 (not 7!)
        self.reg.p.set(StatusFlags::NEGATIVE, bit6);
        self.reg.p.set(StatusFlags::ZERO, result == 0);
        
        Ok(2)
    }
    
    /// LAX - Load Accumulator and X Register with Memory
    /// 
    /// This is an undocumented instruction that loads both the accumulator and X register
    /// with the same value from memory.
    /// 
    /// # Opcodes
    /// - 0xA7: LAX Zero Page
    /// - 0xB7: LAX Zero Page,Y
    /// - 0xAF: LAX Absolute
    /// - 0xBF: LAX Absolute,Y
    /// - 0xA3: LAX (Indirect,X)
    /// - 0xB3: LAX (Indirect),Y
    /// 
    /// # Flags Affected
    /// - N: Set if the loaded value is negative
    /// - Z: Set if the loaded value is zero
    pub(crate) fn lax(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr)?;
        
        // Load value into both A and X registers
        self.reg.a = value;
        self.reg.x = value;
        
        // Update flags
        self.update_zero_and_negative_flags(value);
        
        // Base cycles
        let mut cycles = match mode {
            AddressingMode::ZeroPage => 3,
            AddressingMode::ZeroPageY => 4,
            AddressingMode::Absolute => 4,
            AddressingMode::AbsoluteY => 4 + (if page_crossed { 1 } else { 0 }),
            AddressingMode::IndirectX => 6,
            AddressingMode::IndirectY => 5 + (if page_crossed { 1 } else { 0 }),
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        };
        
        cycles += extra_cycles as u8;
        Ok(cycles)
    }
    
    /// SAX - Store A & X in Memory
    /// 
    /// This is an undocumented instruction that stores the bitwise AND of the accumulator
    /// and X register into memory.
    /// 
    /// # Opcodes
    /// - 0x87: SAX Zero Page
    /// - 0x97: SAX Zero Page,Y
    /// - 0x8F: SAX Absolute
    /// - 0x83: SAX (Indirect,X)
    /// 
    /// # Flags Affected
    /// - None
    pub(crate) fn sax(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _, _) = self.get_operand_address(mode, memory)?;
        let value = self.reg.a & self.reg.x;
        
        memory.write_byte(addr, value)?;
        
        // Base cycles
        let cycles = match mode {
            AddressingMode::ZeroPage => 3,
            AddressingMode::ZeroPageY => 4,
            AddressingMode::Absolute => 4,
            AddressingMode::IndirectX => 6,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        };
        
        Ok(cycles)
    }
    
    /// SLO (ASO) - Arithmetic Shift Left then OR with Accumulator
    /// 
    /// This is an undocumented instruction that performs an ASL on a memory location
    /// and then ORs the result with the accumulator.
    /// 
    /// # Opcodes
    /// - 0x07: SLO Zero Page
    /// - 0x17: SLO Zero Page,X
    /// - 0x0F: SLO Absolute
    /// - 0x1F: SLO Absolute,X
    /// - 0x1B: SLO Absolute,Y
    /// - 0x03: SLO (Indirect,X)
    /// - 0x13: SLO (Indirect),Y
    /// 
    /// # Flags Affected
    /// - N: Set if bit 7 of the result is set
    /// - Z: Set if the result is zero
    /// - C: Set to the value of bit 7 of the original value
    pub(crate) fn slo(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr)?;
        
        // Perform ASL operation
        let result = value << 1;
        memory.write_byte(addr, result)?;
        
        // Set carry flag to bit 7 of original value
        self.reg.p.set(StatusFlags::CARRY, (value & 0x80) != 0);
        
        // OR with accumulator
        self.reg.a |= result;
        
        // Update flags
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Base cycles
        let mut cycles = match mode {
            AddressingMode::ZeroPage => 5,
            AddressingMode::ZeroPageX => 6,
            AddressingMode::Absolute => 6,
            AddressingMode::AbsoluteX => 7,
            AddressingMode::AbsoluteY => 7,
            AddressingMode::IndirectX => 8,
            AddressingMode::IndirectY => 8,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        };
        
        cycles += extra_cycles as u8;
        Ok(cycles)
    }
    
    /// RLA - Rotate Left then AND with Accumulator
    /// 
    /// This is an undocumented instruction that performs a ROL on a memory location
    /// and then ANDs the result with the accumulator.
    /// 
    /// # Opcodes
    /// - 0x27: RLA Zero Page
    /// - 0x37: RLA Zero Page,X
    /// - 0x2F: RLA Absolute
    /// - 0x3F: RLA Absolute,X
    /// - 0x3B: RLA Absolute,Y
    /// - 0x23: RLA (Indirect,X)
    /// - 0x33: RLA (Indirect),Y
    /// 
    /// # Flags Affected
    /// - N: Set if bit 7 of the result is set
    /// - Z: Set if the result is zero
    /// - C: Set to the value of bit 7 of the original value
    pub(crate) fn rla(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr)?;
        
        // Perform ROL operation
        let carry = self.reg.p.contains(StatusFlags::CARRY) as u8;
        let result = (value << 1) | carry;
        memory.write_byte(addr, result)?;
        
        // Set carry flag to bit 7 of original value
        self.reg.p.set(StatusFlags::CARRY, (value & 0x80) != 0);
        
        // AND with accumulator
        self.reg.a &= result;
        
        // Update flags
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Base cycles
        let mut cycles = match mode {
            AddressingMode::ZeroPage => 5,
            AddressingMode::ZeroPageX => 6,
            AddressingMode::Absolute => 6,
            AddressingMode::AbsoluteX => 7,
            AddressingMode::AbsoluteY => 7,
            AddressingMode::IndirectX => 8,
            AddressingMode::IndirectY => 8,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        };
        
        cycles += extra_cycles as u8;
        Ok(cycles)
    }
    
    /// SRE (LSE) - Logical Shift Right then EOR with Accumulator
    /// 
    /// This is an undocumented instruction that performs an LSR on a memory location
    /// and then XORs the result with the accumulator.
    /// 
    /// # Opcodes
    /// - 0x47: SRE Zero Page
    /// - 0x57: SRE Zero Page,X
    /// - 0x4F: SRE Absolute
    /// - 0x5F: SRE Absolute,X
    /// - 0x5B: SRE Absolute,Y
    /// - 0x43: SRE (Indirect,X)
    /// - 0x53: SRE (Indirect),Y
    /// 
    /// # Flags Affected
    /// - N: Set if bit 7 of the result is set
    /// - Z: Set if the result is zero
    /// - C: Set to the value of bit 0 of the original value
    pub(crate) fn sre(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr)?;
        
        // Perform LSR operation
        let result = value >> 1;
        memory.write_byte(addr, result)?;
        
        // Set carry flag to bit 0 of original value
        self.reg.p.set(StatusFlags::CARRY, (value & 0x01) != 0);
        
        // XOR with accumulator
        self.reg.a ^= result;
        
        // Update flags
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Base cycles
        let mut cycles = match mode {
            AddressingMode::ZeroPage => 5,
            AddressingMode::ZeroPageX => 6,
            AddressingMode::Absolute => 6,
            AddressingMode::AbsoluteX => 7,
            AddressingMode::AbsoluteY => 7,
            AddressingMode::IndirectX => 8,
            AddressingMode::IndirectY => 8,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        };
        
        cycles += extra_cycles as u8;
        Ok(cycles)
    }
    
    /// RRA - Rotate Right then Add with Carry
    /// 
    /// This is an undocumented instruction that performs an ROR on a memory location
    /// and then adds the result to the accumulator with carry.
    /// 
    /// # Opcodes
    /// - 0x67: RRA Zero Page
    /// - 0x77: RRA Zero Page,X
    /// - 0x6F: RRA Absolute
    /// - 0x7F: RRA Absolute,X
    /// - 0x7B: RRA Absolute,Y
    /// - 0x63: RRA (Indirect,X)
    /// - 0x73: RRA (Indirect),Y
    /// 
    /// # Flags Affected
    /// - N: Set if bit 7 of the result is set
    /// - Z: Set if the result is zero
    /// - C: Set if the addition resulted in a carry
    /// - V: Set if signed overflow occurred during addition
    pub(crate) fn rra(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr)?;
        
        // Perform ROR operation
        let carry = self.reg.p.contains(StatusFlags::CARRY) as u8;
        let result = (value >> 1) | (carry << 7);
        memory.write_byte(addr, result)?;
        
        // Set carry flag to bit 0 of original value
        self.reg.p.set(StatusFlags::CARRY, (value & 0x01) != 0);
        
        // Add with carry to accumulator
        self.adc_impl(result)?;
        
        // Base cycles
        let mut cycles = match mode {
            AddressingMode::ZeroPage => 5,
            AddressingMode::ZeroPageX => 6,
            AddressingMode::Absolute => 6,
            AddressingMode::AbsoluteX => 7,
            AddressingMode::AbsoluteY => 7,
            AddressingMode::IndirectX => 8,
            AddressingMode::IndirectY => 8,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        };
        
        cycles += extra_cycles as u8;
        Ok(cycles)
    }
    
    /// DCP (DCM) - Decrement Memory then Compare with Accumulator
    /// 
    /// This is an undocumented instruction that decrements a memory location
    /// and then compares it with the accumulator.
    /// 
    /// # Opcodes
    /// - 0xC7: DCP Zero Page
    /// - 0xD7: DCP Zero Page,X
    /// - 0xCF: DCP Absolute
    /// - 0xDF: DCP Absolute,X
    /// - 0xDB: DCP Absolute,Y
    /// - 0xC3: DCP (Indirect,X)
    /// - 0xD3: DCP (Indirect),Y
    /// 
    /// # Flags Affected
    /// - N: Set if the result is negative
    /// - Z: Set if the result is zero
    /// - C: Set if the accumulator is greater than or equal to the result
    pub(crate) fn dcp(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr)?;
        
        // Decrement memory
        let result = value.wrapping_sub(1);
        memory.write_byte(addr, result)?;
        
        // Compare with accumulator
        let diff = self.reg.a.wrapping_sub(result);
        
        // Update flags
        self.reg.p.set(StatusFlags::CARRY, self.reg.a >= result);
        self.update_zero_and_negative_flags(diff);
        
        // Base cycles
        let mut cycles = match mode {
            AddressingMode::ZeroPage => 5,
            AddressingMode::ZeroPageX => 6,
            AddressingMode::Absolute => 6,
            AddressingMode::AbsoluteX => 7,
            AddressingMode::AbsoluteY => 7,
            AddressingMode::IndirectX => 8,
            AddressingMode::IndirectY => 8,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        };
        
        cycles += extra_cycles as u8;
        Ok(cycles)
    }
    
    /// ISC (ISB) - Increment Memory then Subtract with Carry
    /// 
    /// This is an undocumented instruction that increments a memory location
    /// and then subtracts it from the accumulator with borrow.
    /// 
    /// # Opcodes
    /// - 0xE7: ISC Zero Page
    /// - 0xF7: ISC Zero Page,X
    /// - 0xEF: ISC Absolute
    /// - 0xFF: ISC Absolute,X
    /// - 0xFB: ISC Absolute,Y
    /// - 0xE3: ISC (Indirect,X)
    /// - 0xF3: ISC (Indirect),Y
    /// 
    /// # Flags Affected
    /// - N: Set if the result is negative
    /// - Z: Set if the result is zero
    /// - C: Set if the subtraction did not require a borrow
    /// - V: Set if signed overflow occurred during subtraction
    pub(crate) fn isc(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr)?;
        
        // Increment memory
        let result = value.wrapping_add(1);
        memory.write_byte(addr, result)?;
        
        // Subtract from accumulator with borrow (SBC)
        let a = self.reg.a as u16;
        let b = result as u16;
        let c = if self.reg.p.contains(StatusFlags::CARRY) { 0 } else { 1 };
        let diff = a.wrapping_sub(b).wrapping_sub(c);
        
        // Update accumulator and flags
        self.reg.a = diff as u8;
        self.reg.p.set(StatusFlags::CARRY, diff <= 0xFF);
        
        // Set overflow flag
        let overflow = ((a ^ b) & 0x80) != 0 && ((a ^ diff) & 0x80) != 0;
        self.reg.p.set(StatusFlags::OVERFLOW, overflow);
        
        // Update zero and negative flags
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Base cycles
        let mut cycles = match mode {
            AddressingMode::ZeroPage => 5,
            AddressingMode::ZeroPageX => 6,
            AddressingMode::Absolute => 6,
            AddressingMode::AbsoluteX => 7,
            AddressingMode::AbsoluteY => 7,
            AddressingMode::IndirectX => 8,
            AddressingMode::IndirectY => 8,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        };
        
        cycles += extra_cycles as u8;
        Ok(cycles)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::cpu::tests::TestMemory;
    
    #[test]
    fn test_aac() {
        let mut cpu = Cpu::new();
        let mut memory = TestMemory::new();
        
        // Test case 1: AND with 0x55, result is 0x55, bit 7 is 0
        cpu.reg.a = 0xFF;
        cpu.reg.pc = 0x8000;
        memory.write_byte(0x8000, 0x55).unwrap();
        
        let cycles = cpu.aac(&mut memory).unwrap();
        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.a, 0x55);
        assert!(!cpu.reg.p.contains(StatusFlags::CARRY));
        assert!(!cpu.reg.p.contains(StatusFlags::ZERO));
        assert!(!cpu.reg.p.contains(StatusFlags::NEGATIVE));
        
        // Test case 2: AND with 0x80, result is 0x80, bit 7 is 1
        cpu.reg.a = 0xFF;
        cpu.reg.pc = 0x8001;
        memory.write_byte(0x8001, 0x80).unwrap();
        
        let cycles = cpu.aac(&mut memory).unwrap();
        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.a, 0x80);
        assert!(cpu.reg.p.contains(StatusFlags::CARRY));
        assert!(!cpu.reg.p.contains(StatusFlags::ZERO));
        assert!(cpu.reg.p.contains(StatusFlags::NEGATIVE));
    }
    
    #[test]
    fn test_lax() {
        let mut cpu = Cpu::new();
        let mut memory = TestMemory::new();
        
        // Test LAX Zero Page
        cpu.reg.pc = 0x8000;
        memory.write_byte(0x8000, 0x42).unwrap();  // Zero page address
        memory.write_byte(0x0042, 0x7F).unwrap();  // Value at 0x42
        
        let cycles = cpu.lax(AddressingMode::ZeroPage, &mut memory).unwrap();
        assert_eq!(cycles, 3);
        assert_eq!(cpu.reg.a, 0x7F);
        assert_eq!(cpu.reg.x, 0x7F);
        assert!(!cpu.reg.p.contains(StatusFlags::ZERO));
        assert!(!cpu.reg.p.contains(StatusFlags::NEGATIVE));
    }
    
    #[test]
    fn test_ane() {
        let mut cpu = Cpu::new();
        let mut memory = TestMemory::new();
        
        // Test ANE with X=0xFF, A=0xFF, immediate value 0xF0
        cpu.reg.x = 0xFF;
        cpu.reg.a = 0xFF;
        cpu.reg.pc = 0x8000;
        memory.write_byte(0x8000, 0xF0).unwrap();
        
        let cycles = cpu.ane(&mut memory).unwrap();
        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.a, 0xF0);  // X & A & imm8
        assert!(!cpu.reg.p.contains(StatusFlags::ZERO));
        assert!(cpu.reg.p.contains(StatusFlags::NEGATIVE));
    }
    
    #[test]
    fn test_arr() {
        let mut cpu = Cpu::new();
        let mut memory = TestMemory::new();
        
        // Test ARR with A=0x81, immediate value 0x81, C=1
        cpu.reg.a = 0x81;
        cpu.reg.p.set(StatusFlags::CARRY, true);
        cpu.reg.pc = 0x8000;
        memory.write_byte(0x8000, 0x81).unwrap();
        
        let cycles = cpu.arr(&mut memory).unwrap();
        assert_eq!(cycles, 2);
        assert_eq!(cpu.reg.a, 0xC0);  // (0x81 & 0x81) >> 1 | 0x80
        assert!(cpu.reg.p.contains(StatusFlags::CARRY));
        assert!(cpu.reg.p.contains(StatusFlags::NEGATIVE));
        assert!(!cpu.reg.p.contains(StatusFlags::ZERO));
        assert!(cpu.reg.p.contains(StatusFlags::OVERFLOW));
    }
    
    #[test]
    fn test_sax() {
        let mut cpu = Cpu::new();
        let mut memory = TestMemory::new();
        
        // Test SAX Zero Page with A=0x55, X=0xAA
        cpu.reg.a = 0x55;
        cpu.reg.x = 0xAA;
        cpu.reg.pc = 0x8000;
        memory.write_byte(0x8000, 0x42).unwrap();  // Zero page address
        
        let cycles = cpu.sax(AddressingMode::ZeroPage, &mut memory).unwrap();
        assert_eq!(cycles, 3);
        assert_eq!(memory.read_byte(0x0042).unwrap(), 0x00);  // 0x55 & 0xAA = 0x00
    }
    
    #[test]
    fn test_slo() {
        let mut cpu = Cpu::new();
        let mut memory = TestMemory::new();
        
        // Test SLO Zero Page with A=0x01, memory=0x41
        cpu.reg.a = 0x01;
        cpu.reg.pc = 0x8000;
        memory.write_byte(0x8000, 0x42).unwrap();  // Zero page address
        memory.write_byte(0x0042, 0x41).unwrap();  // Value to shift left
        
        let cycles = cpu.slo(AddressingMode::ZeroPage, &mut memory).unwrap();
        assert_eq!(cycles, 5);
        assert_eq!(memory.read_byte(0x0042).unwrap(), 0x82);  // 0x41 << 1 = 0x82
        assert_eq!(cpu.reg.a, 0x83);  // 0x01 | 0x82 = 0x83
        assert!(cpu.reg.p.contains(StatusFlags::NEGATIVE));
        assert!(!cpu.reg.p.contains(StatusFlags::ZERO));
        assert!(!cpu.reg.p.contains(StatusFlags::CARRY));
    }
    
    #[test]
    fn test_dcp() {
        let mut cpu = Cpu::new();
        let mut memory = TestMemory::new();
        
        // Test DCP Zero Page with A=0x42, memory=0x41
        cpu.reg.a = 0x42;
        cpu.reg.pc = 0x8000;
        memory.write_byte(0x8000, 0x42).unwrap();  // Zero page address
        memory.write_byte(0x0042, 0x41).unwrap();  // Value to decrement
        
        let cycles = cpu.dcp(AddressingMode::ZeroPage, &mut memory).unwrap();
        assert_eq!(cycles, 5);
        assert_eq!(memory.read_byte(0x0042).unwrap(), 0x40);  // 0x41 - 1 = 0x40
        assert!(cpu.reg.p.contains(StatusFlags::CARRY));  // 0x42 >= 0x40
        assert!(!cpu.reg.p.contains(StatusFlags::ZERO));
        assert!(!cpu.reg.p.contains(StatusFlags::NEGATIVE));
    }
    
    #[test]
    fn test_isc() {
        let mut cpu = Cpu::new();
        let mut memory = TestMemory::new();
        
        // Test ISC Zero Page with A=0x42, memory=0x41, C=1
        cpu.reg.a = 0x42;
        cpu.reg.p.set(StatusFlags::CARRY, true);
        cpu.reg.pc = 0x8000;
        memory.write_byte(0x8000, 0x42).unwrap();  // Zero page address
        memory.write_byte(0x0042, 0x41).unwrap();  // Value to increment and subtract
        
        let cycles = cpu.isc(AddressingMode::ZeroPage, &mut memory).unwrap();
        assert_eq!(cycles, 5);
        assert_eq!(memory.read_byte(0x0042).unwrap(), 0x42);  // 0x41 + 1 = 0x42
        assert_eq!(cpu.reg.a, 0x00);  // 0x42 - 0x42 = 0x00
        assert!(cpu.reg.p.contains(StatusFlags::ZERO));
        assert!(!cpu.reg.p.contains(StatusFlags::NEGATIVE));
        assert!(cpu.reg.p.contains(StatusFlags::CARRY));
    }
}
