use super::{Cpu, CpuResult, CpuError, AddressingMode};
use super::status_flags::StatusFlags;
use crate::nes::utils::Memory;

impl Cpu {
    // =============================================
    // Flag Operations (from unofficial_ops.rs)
    // =============================================
    
    /// CLC - Clear Carry Flag
    pub(crate) fn clc(&mut self) -> CpuResult<u8> {
        self.reg.p.remove(StatusFlags::CARRY);
        Ok(2)
    }

    /// CLD - Clear Decimal Mode
    pub(crate) fn cld(&mut self) -> CpuResult<u8> {
        self.reg.p.remove(StatusFlags::DECIMAL);
        Ok(2)
    }

    /// CLI - Clear Interrupt Disable
    pub(crate) fn cli(&mut self) -> CpuResult<u8> {
        self.reg.p.remove(StatusFlags::INTERRUPT_DISABLE);
        Ok(2)
    }

    /// CLV - Clear Overflow Flag
    pub(crate) fn clv(&mut self) -> CpuResult<u8> {
        self.reg.p.remove(StatusFlags::OVERFLOW);
        Ok(2)
    }

    /// SEC - Set Carry Flag
    pub(crate) fn sec(&mut self) -> CpuResult<u8> {
        self.reg.p.insert(StatusFlags::CARRY);
        Ok(2)
    }

    /// SED - Set Decimal Flag
    pub(crate) fn sed(&mut self) -> CpuResult<u8> {
        self.reg.p.insert(StatusFlags::DECIMAL);
        Ok(2)
    }

    /// SEI - Set Interrupt Disable
    pub(crate) fn sei(&mut self) -> CpuResult<u8> {
        self.reg.p.insert(StatusFlags::INTERRUPT_DISABLE);
        Ok(2)
    }

    /// BRK - Force Interrupt
    pub(crate) fn brk(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        // Push PC + 2 to stack (PC points to next instruction after BRK)
        let pc_plus_2 = self.reg.pc.wrapping_add(1);
        self.push_word(memory, pc_plus_2)?;
        
        // Push status with B flag set
        let mut status = self.reg.p;
        status.insert(StatusFlags::BREAK);
        self.push_byte(memory, status.bits())?;
        
        // Set interrupt disable flag
        self.reg.p.insert(StatusFlags::INTERRUPT_DISABLE);
        
        // Jump to interrupt vector
        let lo = self.read_byte(memory, 0xFFFE)? as u16;
        let hi = self.read_byte(memory, 0xFFFF)? as u16;
        self.reg.pc = (hi << 8) | lo;
        
        Ok(7) // BRK takes 7 cycles
    }

    // =============================================
    // Memory Operations (from unofficial_impls.rs)
    // =============================================
    
    /// SHA (SHX, SXA, XAS) - Store A & X & (high byte of address + 1)
    pub(crate) fn sha(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _page_crossed, _) = self.get_operand_address(mode, memory)?;
        
        // Calculate the high byte of the address + 1
        let high_byte_plus_1 = ((addr >> 8) + 1) as u8;
        
        // Calculate the value to store: A & X & (high_byte + 1)
        let value = self.reg.a & self.reg.x & high_byte_plus_1;
        
        // Write the value to memory
        memory.write_byte(addr, value).map_err(CpuError::MemoryError)?;
        
        // Determine cycle count based on addressing mode and page crossing
        let cycles = match mode {
            AddressingMode::AbsoluteY => 5,
            AddressingMode::IndirectY => 6,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        };
        
        Ok(cycles)
    }
    
    /// SHY (SHY, SYA) - Store Y & (high byte of address + 1)
    pub(crate) fn shy(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _page_crossed, _) = self.get_operand_address(mode, memory)?;
        
        // Calculate the high byte of the address + 1
        let high_byte_plus_1 = ((addr >> 8) + 1) as u8;
        
        // Calculate the value to store: Y & (high_byte + 1)
        let value = self.reg.y & high_byte_plus_1;
        
        // Write the value to memory
        memory.write_byte(addr, value).map_err(CpuError::MemoryError)?;
        
        // SHY only supports Absolute,X addressing
        match mode {
            AddressingMode::AbsoluteX => Ok(5),
            _ => Err(CpuError::InvalidAddressingMode(mode)),
        }
    }
    
    /// SHX (SHX, SXA) - Store X & (high byte of address + 1)
    pub(crate) fn shx(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _page_crossed, _) = self.get_operand_address(mode, memory)?;
        
        // Calculate the high byte of the address + 1
        let high_byte_plus_1 = ((addr >> 8) + 1) as u8;
        
        // Calculate the value to store: X & (high_byte + 1)
        let value = self.reg.x & high_byte_plus_1;
        
        // Write the value to memory
        memory.write_byte(addr, value).map_err(CpuError::MemoryError)?;
        
        // SHX only supports Absolute,Y addressing
        match mode {
            AddressingMode::AbsoluteY => Ok(5),
            _ => Err(CpuError::InvalidAddressingMode(mode)),
        }
    }
    
    /// SXA (SHX) - Alias for SHX
    pub(crate) fn sxa(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        self.shx(mode, memory)
    }
    
    /// SYA (SHY) - Alias for SHY
    pub(crate) fn sya(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        self.shy(mode, memory)
    }
    
    // =============================================
    // ALU Operations (from more_unofficial.rs)
    // =============================================
    
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
        let (addr, _page_crossed, _) = self.get_operand_address(mode, memory)?;
        let value = memory.read_byte(addr).map_err(CpuError::MemoryError)?;
        
        // AND memory with stack pointer
        let result = value & self.reg.s;
        
        // Store result in A, X, and S
        self.reg.a = result;
        self.reg.x = result;
        self.reg.s = result;
        
        // Update flags
        self.update_zero_and_negative_flags(result);
        
        // Return cycle count based on addressing mode
        match mode {
            AddressingMode::AbsoluteY => Ok(4),
            _ => Err(CpuError::InvalidAddressingMode(mode)),
        }
    }
    
    // =============================================
    // Shift and Rotate Operations
    // =============================================
    
    /// SLO (ASO) - Arithmetic Shift Left then OR with Accumulator (Unofficial)
    /// Shifts the value in memory left by 1 bit, then ORs the result with the accumulator.
    pub(crate) fn slo(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _page_crossed, _extra_cycles) = self.get_operand_address(mode, memory)?;
        let mut value = memory.read_byte(addr).map_err(CpuError::MemoryError)?;
        
        // Shift left and update carry flag
        self.reg.p.set(StatusFlags::CARRY, (value & 0x80) != 0);
        value = value.wrapping_shl(1);
        
        // Write back the shifted value
        memory.write_byte(addr, value).map_err(CpuError::MemoryError)?;
        
        // OR with accumulator
        self.reg.a |= value;
        
        // Update flags
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Return cycle count based on addressing mode
        match mode {
            AddressingMode::ZeroPage => Ok(5),
            AddressingMode::ZeroPageX => Ok(6),
            AddressingMode::Absolute => Ok(6),
            AddressingMode::AbsoluteX => Ok(7),
            AddressingMode::AbsoluteY => Ok(7),
            AddressingMode::IndirectX => Ok(8),
            AddressingMode::IndirectY => Ok(8),
            _ => Err(CpuError::InvalidAddressingMode(mode)),
        }
    }
    
    /// SRE (LSE) - Logical Shift Right then EOR with Accumulator (Unofficial)
    /// Shifts the value in memory right by 1 bit, then XORs the result with the accumulator.
    pub(crate) fn sre(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _page_crossed, _extra_cycles) = self.get_operand_address(mode, memory)?;
        let mut value = memory.read_byte(addr).map_err(CpuError::MemoryError)?;
        
        // Shift right and update carry flag
        self.reg.p.set(StatusFlags::CARRY, (value & 0x01) != 0);
        value = value.wrapping_shr(1);
        
        // Write back the shifted value
        memory.write_byte(addr, value).map_err(CpuError::MemoryError)?;
        
        // XOR with accumulator
        self.reg.a ^= value;
        
        // Update flags
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Return cycle count based on addressing mode
        match mode {
            AddressingMode::ZeroPage => Ok(5),
            AddressingMode::ZeroPageX => Ok(6),
            AddressingMode::Absolute => Ok(6),
            AddressingMode::AbsoluteX => Ok(7),
            AddressingMode::AbsoluteY => Ok(7),
            AddressingMode::IndirectX => Ok(8),
            AddressingMode::IndirectY => Ok(8),
            _ => Err(CpuError::InvalidAddressingMode(mode)),
        }
    }
    
    /// RLA - Rotate Left then AND with Accumulator (Unofficial)
    /// Rotates the value in memory left by 1 bit (including carry), then ANDs with accumulator.
    pub(crate) fn rla(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _page_crossed, _extra_cycles) = self.get_operand_address(mode, memory)?;
        let mut value = memory.read_byte(addr).map_err(CpuError::MemoryError)?;
        
        // Rotate left through carry
        let new_carry = (value & 0x80) != 0;
        let carry_bit = if self.reg.p.contains(StatusFlags::CARRY) { 1 } else { 0 };
        value = (value << 1) | carry_bit;
        
        // Update carry flag
        self.reg.p.set(StatusFlags::CARRY, new_carry);
        
        // Write back the rotated value
        memory.write_byte(addr, value).map_err(CpuError::MemoryError)?;
        
        // AND with accumulator
        self.reg.a &= value;
        
        // Update flags
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Return cycle count based on addressing mode
        match mode {
            AddressingMode::ZeroPage => Ok(5),
            AddressingMode::ZeroPageX => Ok(6),
            AddressingMode::Absolute => Ok(6),
            AddressingMode::AbsoluteX => Ok(7),
            AddressingMode::AbsoluteY => Ok(7),
            AddressingMode::IndirectX => Ok(8),
            AddressingMode::IndirectY => Ok(8),
            _ => Err(CpuError::InvalidAddressingMode(mode)),
        }
    }
    
    /// RRA - Rotate Right then Add with Carry (Unofficial)
    /// Rotates the value in memory right by 1 bit (including carry), then adds to accumulator with carry.
    pub(crate) fn rra(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _page_crossed, _extra_cycles) = self.get_operand_address(mode, memory)?;
        let mut value = memory.read_byte(addr).map_err(CpuError::MemoryError)?;
        
        // Rotate right through carry
        let _new_carry = (value & 0x01) != 0;
        let carry_bit = if self.reg.p.contains(StatusFlags::CARRY) { 0x80 } else { 0 };
        value = (value >> 1) | carry_bit;
        
        // Write back the rotated value
        memory.write_byte(addr, value).map_err(CpuError::MemoryError)?;
        
        // Add with carry to accumulator (reusing the ADC implementation)
        let result = self.add_with_carry(value);
        self.reg.a = result as u8;
        
        // Update carry flag from the addition
        self.reg.p.set(StatusFlags::CARRY, result > 0xFF);
        
        // Update overflow flag (copied from ADC implementation)
        let overflow = (!(self.reg.a as i8 ^ value as i8) & (self.reg.a as i8 ^ result as i8)) < 0;
        self.reg.p.set(StatusFlags::OVERFLOW, overflow);
        
        // Update zero and negative flags
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Return cycle count based on addressing mode
        match mode {
            AddressingMode::ZeroPage => Ok(5),
            AddressingMode::ZeroPageX => Ok(6),
            AddressingMode::Absolute => Ok(6),
            AddressingMode::AbsoluteX => Ok(7),
            AddressingMode::AbsoluteY => Ok(7),
            AddressingMode::IndirectX => Ok(8),
            AddressingMode::IndirectY => Ok(8),
            _ => Err(CpuError::InvalidAddressingMode(mode)),
        }
    }
    
    // =============================================
    // Other Unofficial Opcodes
    // =============================================
    
    /// AAC (ANC) - AND byte with accumulator, then move bit 7 to Carry
    pub(crate) fn aac(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        let value = self.read_byte(memory, self.reg.pc)?;
        self.reg.pc = self.reg.pc.wrapping_add(1);
        
        self.reg.a &= value;
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Set carry to bit 7 of the result
        self.reg.p.set(StatusFlags::CARRY, (self.reg.a & 0x80) != 0);
        
        Ok(2)
    }
    
    /// ANE (XAA) - Exact operation varies by hardware, this is a common implementation
    pub(crate) fn ane(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        let value = self.read_byte(memory, self.reg.pc)?;
        self.reg.pc = self.reg.pc.wrapping_add(1);
        
        // This is a simplified implementation; actual behavior varies
        self.reg.a = self.reg.x & self.reg.a & value;
        self.update_zero_and_negative_flags(self.reg.a);
        
        Ok(2)
    }
    
    /// ARR - AND byte with accumulator, then rotate right and check bits 5 and 6
    pub(crate) fn arr(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        let value = self.read_byte(memory, self.reg.pc)?;
        self.reg.pc = self.reg.pc.wrapping_add(1);
        
        // AND with accumulator
        let mut result = self.reg.a & value;
        
        // Rotate right through carry
        let carry = self.reg.p.contains(StatusFlags::CARRY);
        self.reg.p.set(StatusFlags::CARRY, (result & 0x01) != 0);
        result = (result >> 1) | if carry { 0x80 } else { 0 };
        
        self.reg.a = result;
        self.update_zero_and_negative_flags(self.reg.a);
        
        // Set overflow flag based on bits 6 and 5
        let v = ((result >> 6) ^ (result >> 5)) & 0x01;
        self.reg.p.set(StatusFlags::OVERFLOW, v != 0);
        
        Ok(2)
    }
    
    // =============================================
    // Other Unofficial Opcodes (continued)
    // =============================================
    
    /// LAX - Load Accumulator and X Register (Unofficial)
    pub(crate) fn lax(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _page_crossed, _extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = self.read_byte(memory, addr)?;
        
        self.reg.a = value;
        self.reg.x = value;
        
        self.update_zero_and_negative_flags(self.reg.a);
        
        match mode {
            AddressingMode::Immediate => Ok(2),
            AddressingMode::ZeroPage => Ok(3),
            AddressingMode::ZeroPageY | AddressingMode::ZeroPageX => Ok(4),
            AddressingMode::Absolute => Ok(4),
            AddressingMode::AbsoluteY | AddressingMode::AbsoluteX => Ok(4 + _page_crossed as u8),
            AddressingMode::IndirectX => Ok(6),
            AddressingMode::IndirectY => Ok(5 + _page_crossed as u8),
            _ => Err(CpuError::InvalidAddressingMode(mode)),
        }
    }
    
    /// SAX - Store A & X (Unofficial)
    pub(crate) fn sax(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _page_crossed, _extra_cycles) = self.get_operand_address(mode, memory)?;
        let value = self.reg.a & self.reg.x;
        
        memory.write_byte(addr, value)?;
        
        match mode {
            AddressingMode::ZeroPage => Ok(3),
            AddressingMode::ZeroPageY => Ok(4),
            AddressingMode::Absolute => Ok(4),
            AddressingMode::IndirectX => Ok(6),
            _ => Err(CpuError::InvalidAddressingMode(mode)),
        }
    }
    
    /// DCP - Decrement Memory then Compare with Accumulator (Unofficial)
    pub(crate) fn dcp(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _page_crossed, _extra_cycles) = self.get_operand_address(mode, memory)?;
        let mut value = self.read_byte(memory, addr)?;
        
        // Decrement memory
        value = value.wrapping_sub(1);
        memory.write_byte(addr, value)?;
        
        // Compare with accumulator (using the compare_register method with Immediate mode since we already have the value)
        self.compare_register(AddressingMode::Immediate, value, memory)?;
        
        match mode {
            AddressingMode::ZeroPage => Ok(5),
            AddressingMode::ZeroPageX => Ok(6),
            AddressingMode::Absolute => Ok(6),
            AddressingMode::AbsoluteX | AddressingMode::AbsoluteY => Ok(7),
            AddressingMode::IndirectX => Ok(8),
            AddressingMode::IndirectY => Ok(8 + if _page_crossed { 1 } else { 0 }),
            _ => Err(CpuError::InvalidAddressingMode(mode)),
        }
    }
    
    /// ISC (ISB) - Increment Memory then Subtract with Carry (Unofficial)
    pub(crate) fn isc(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _page_crossed, _extra_cycles) = self.get_operand_address(mode, memory)?;
        let mut value = self.read_byte(memory, addr)?;
        
        // Increment memory
        value = value.wrapping_add(1);
        memory.write_byte(addr, value)?;
        
        // Subtract with borrow (SBC)
        // Since we already have the value, we can use the adc_impl with the value negated and carry inverted
        let carry = self.reg.p.contains(StatusFlags::CARRY);
        self.adc_impl(!value)?;
        
        // Invert the carry flag since we're doing subtraction
        self.reg.p.set(StatusFlags::CARRY, !carry);
        
        match mode {
            AddressingMode::ZeroPage => Ok(5),
            AddressingMode::ZeroPageX => Ok(6),
            AddressingMode::Absolute => Ok(6),
            AddressingMode::AbsoluteX | AddressingMode::AbsoluteY => Ok(7),
            AddressingMode::IndirectX => Ok(8),
            AddressingMode::IndirectY => Ok(8 + _page_crossed as u8),
            _ => Err(CpuError::InvalidAddressingMode(mode)),
        }
    }
    
    /// XAS (SHX) - AND X register with the high byte of the target address + 1
    /// then AND with A, store in memory
    pub(crate) fn xas(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, _page_crossed, _extra_cycles) = self.get_operand_address(mode, memory)?;
        
        // The exact behavior varies between CPU revisions
        // This is a common implementation that works for most cases
        let value = self.reg.x & self.reg.a & ((addr >> 8) as u8).wrapping_add(1);
        
        // Write the result to memory
        memory.write_byte(addr, value).map_err(CpuError::MemoryError)?;
        
        // Update flags
        self.update_zero_and_negative_flags(value);
        
        // Return cycle count (base + page crossing penalty if any)
        match mode {
            AddressingMode::AbsoluteX | AddressingMode::AbsoluteY => Ok(5 + extra_cycles),
            _ => Ok(5),
        }
    }
    
    /// Helper method for ADC operations
    /// Returns the result of A + value + carry
    fn add_with_carry(&self, value: u8) -> u16 {
        let carry = if self.reg.p.contains(StatusFlags::CARRY) { 1 } else { 0 };
        self.reg.a as u16 + value as u16 + carry as u16
    }
    
    /// NOP - No Operation (Unofficial/Illegal)
    /// This is a placeholder for unimplemented unofficial opcodes.
    /// It does nothing and takes 2 cycles.
    pub(crate) fn nop(&mut self, _memory: &mut impl Memory) -> CpuResult<u8> {
        Ok(2)
    }
}
