use super::{Cpu, CpuResult, AddressingMode, StatusFlags};
use crate::nes::utils::Memory;

impl Cpu {
    /// SHA (SHX, SXA, XAS) - Store A & X & (high byte of address + 1)
    /// 
    /// This is an undocumented instruction that stores the bitwise AND of the accumulator,
    /// X register, and the high byte of the target address + 1 into memory.
    /// 
    /// # Opcodes
    /// - 0x9E: SHA (SHX) Absolute,Y
    /// - 0x9F: SHA (SHX) Absolute,Y (alternate)
    /// - 0x93: SHA (SHX) (Indirect),Y
    /// 
    /// # Flags Affected
    /// - None
    pub(crate) fn sha(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, _) = self.get_operand_address(mode, memory)?;
        
        // Calculate the high byte of the address + 1
        let high_byte_plus_1 = ((addr >> 8) + 1) as u8;
        
        // Calculate the value to store: A & X & (high_byte + 1)
        let value = self.reg.a & self.reg.x & high_byte_plus_1;
        
        // Write the value to memory
        memory.write_byte(addr, value).map_err(|e| CpuError::MemoryError(e))?;
        
        // Determine cycle count based on addressing mode and page crossing
        let cycles = match mode {
            AddressingMode::AbsoluteY => 5,
            AddressingMode::IndirectY => 6,
            _ => return Err(CpuError::InvalidAddressingMode(mode)),
        };
        
        Ok(cycles)
    }
    
    /// SHY (SHY, SYA) - Store Y & (high byte of address + 1)
    /// 
    /// This is an undocumented instruction that stores the bitwise AND of the Y register
    /// and the high byte of the target address + 1 into memory.
    /// 
    /// # Opcodes
    /// - 0x9C: SHY (SHY) Absolute,X
    /// 
    /// # Flags Affected
    /// - None
    pub(crate) fn shy(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, _) = self.get_operand_address(mode, memory)?;
        let high_byte_plus_1 = ((addr >> 8) + 1) as u8;
        let value = self.reg.y & high_byte_plus_1;
        memory.write_byte(addr, value).map_err(CpuError::MemoryError)?;
        
        // Return cycle count based on addressing mode
        match mode {
            AddressingMode::AbsoluteX => Ok(5),
            _ => Err(CpuError::InvalidAddressingMode(mode)),
        }
    }
    
    /// SHX (SHX, SXA) - Store X & (high byte of address + 1)
    /// 
    /// This is an undocumented instruction that stores the bitwise AND of the X register
    /// and the high byte of the target address + 1 into memory.
    /// 
    /// # Opcodes
    /// - 0x9E: SHX (SXA) Absolute,Y
    /// 
    /// # Flags Affected
    /// - None
    pub(crate) fn shx(&mut self, mode: AddressingMode, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, _) = self.get_operand_address(mode, memory)?;
        let high_byte_plus_1 = ((addr >> 8) + 1) as u8;
        let value = self.reg.x & high_byte_plus_1;
        memory.write_byte(addr, value).map_err(CpuError::MemoryError)?;
        
        // Return cycle count based on addressing mode
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
    
    /// AXS (SBX) - AND X register with accumulator and store in X, then subtract memory from X
    /// 
    /// This is an undocumented instruction that performs an AND between A and X, stores the result in X,
    /// then subtracts a memory value from X (without borrow).
    /// 
    /// # Opcodes
    /// - 0xCB: AXS (SBX) Immediate
    /// 
    /// # Flags Affected
    /// - N: Set if result is negative
    /// - Z: Set if result is zero
    /// - C: Set if no borrow was needed (X >= value)
    pub(crate) fn axs(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        let value = memory.read_byte(self.reg.pc + 1).map_err(CpuError::MemoryError)?;
        
        // AND A and X, store in X
        self.reg.x = self.reg.a & self.reg.x;
        
        // Compare X with memory value
        let result = self.reg.x.wrapping_sub(value);
        
        // Update flags
        self.reg.p.set(StatusFlags::CARRY, self.reg.x >= value);
        self.reg.p.set(StatusFlags::ZERO, result == 0);
        self.reg.p.set(StatusFlags::NEGATIVE, (result & 0x80) != 0);
        
        // Store result in X
        self.reg.x = result;
        
        // Skip the immediate byte
        self.reg.pc = self.reg.pc.wrapping_add(1);
        
        Ok(2)
    }
    
    /// DOP (NOP) - Double NOP (2-byte NOP)
    /// 
    /// This is an undocumented instruction that does nothing for 2 cycles.
    /// 
    /// # Opcodes
    /// - 0xE2: DOP (NOP) Immediate
    /// 
    /// # Flags Affected
    /// - None
    pub(crate) fn dop(&mut self, _memory: &mut impl Memory) -> CpuResult<u8> {
        // Just increment PC past the immediate byte
        self.reg.pc = self.reg.pc.wrapping_add(1);
        Ok(2)
    }
    
    /// STP (KIL) - Stop the processor
    /// 
    /// This is an undocumented instruction that halts the CPU until a reset.
    /// 
    /// # Opcodes
    /// - 0x02: STP (KIL)
    /// - 0x12: STP (KIL)
    /// - 0x22: STP (KIL)
    /// - 0x32: STP (KIL)
    /// - 0x42: STP (KIL)
    /// - 0x52: STP (KIL)
    /// - 0x62: STP (KIL)
    /// - 0x72: STP (KIL)
    /// - 0x92: STP (KIL)
    /// - 0xB2: STP (KIL)
    /// - 0xD2: STP (KIL)
    /// - 0xF2: STP (KIL)
    /// 
    /// # Flags Affected
    /// - None
    pub(crate) fn stp(&mut self) -> CpuResult<u8> {
        // Halt the CPU
        self.is_running = false;
        Ok(0) // Cycles don't matter since CPU is halted
    }
    
    /// LAS (LAR) - AND memory with stack pointer, transfer to A, X, and S
    /// 
    /// This is an undocumented instruction that performs a bitwise AND between a memory value
    /// and the stack pointer, then stores the result in A, X, and S.
    /// 
    /// # Opcodes
    /// - 0xBB: LAS (LAR) Absolute,Y
    /// 
    /// # Flags Affected
    /// - N: Set if result is negative
    /// - Z: Set if result is zero
    pub(crate) fn las(&mut self, memory: &mut impl Memory) -> CpuResult<u8> {
        let (addr, page_crossed, _) = self.get_operand_address(AddressingMode::AbsoluteY, memory)?;
        let value = memory.read_byte(addr).map_err(CpuError::MemoryError)?;
        
        // AND memory with stack pointer, store in A, X, and S
        let result = value & self.reg.s;
        self.reg.a = result;
        self.reg.x = result;
        self.reg.s = result;
        
        // Update flags
        self.reg.p.set(StatusFlags::ZERO, result == 0);
        self.reg.p.set(StatusFlags::NEGATIVE, (result & 0x80) != 0);
        
        // Return cycle count (4 + 1 if page crossed)
        Ok(4 + if page_crossed { 1 } else { 0 })
    }
}
