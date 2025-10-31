use super::{Cpu, CpuResult, CpuError, AddressingMode, StatusFlags};
use crate::nes::utils::Memory;

impl Cpu {
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
        
        // Push status with Break flag set
        let mut status = self.reg.p;
        status.insert(StatusFlags::BREAK);
        status.insert(StatusFlags::UNUSED); // Bit 5 is always set when pushed to stack
        self.push_byte(memory, status.bits())?;
        
        // Set interrupt disable flag
        self.reg.p.insert(StatusFlags::INTERRUPT_DISABLE);
        
        // Jump to IRQ/BRK vector (0xFFFE/F)
        let irq_vector = self.read_word(memory, 0xFFFE)?;
        self.reg.pc = irq_vector;
        
        Ok(7)
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
