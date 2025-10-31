use bitflags::bitflags;

bitflags! {
    /// # CPU Status Register (P) Flags
    ///
    /// The status register contains 7 flags that indicate the current state of the CPU.
    /// The 5th bit is unused but always set to 1 when pushed to the stack.
    ///
    /// | Bit | Mask | Name       | Description                                      |
    /// |-----|------|------------|--------------------------------------------------|
    /// | 7   | 0x80 | NEGATIVE   | Set when the result is negative                 |
    /// | 6   | 0x40 | OVERFLOW   | Set when an overflow occurs                      |
    /// | 5   | 0x20 | UNUSED     | Always set to 1 when pushed to stack             |
    /// | 4   | 0x10 | BREAK      | Set when a BRK instruction was executed          |
    /// | 3   | 0x08 | DECIMAL    | Decimal mode flag (not used in NES)              |
    /// | 2   | 0x04 | INTERRUPT_DISABLE | When set, disables maskable interrupts    |
    /// | 1   | 0x02 | ZERO       | Set when the result is zero                      |
    /// | 0   | 0x01 | CARRY      | Set when an operation results in a carry/borrow  |
    #[derive(Default, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
    pub struct StatusFlags: u8 {
        /// Carry Flag
        const CARRY = 0b0000_0001;
        /// Zero Flag
        const ZERO = 0b0000_0010;
        /// Interrupt Disable
        const INTERRUPT_DISABLE = 0b0000_0100;
        /// Decimal Mode (not used in NES)
        const DECIMAL = 0b0000_1000;
        /// Break Command
        const BREAK = 0b0001_0000;
        /// Unused (always 1 when pushed to stack)
        const UNUSED = 0b0010_0000;
        /// Overflow Flag
        const OVERFLOW = 0b0100_0000;
        /// Negative Flag
        const NEGATIVE = 0b1000_0000;
    }
}
