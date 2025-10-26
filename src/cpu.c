#include "cpu.h"
#include <stdbool.h>
#include <stddef.h>  // For NULL
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Forward declarations for functions used before they're defined
static void cpu_lda(Cpu *cpu, uint8_t value);
static void cpu_nmi(Cpu *cpu);
static void cpu_irq(Cpu *cpu);

// Helper macros for flag operations
#define SET_FLAG(flag, cond) cpu->p = (cond) ? (cpu->p | (flag)) : (cpu->p & ~(flag))
#define GET_FLAG(flag) (cpu->p & (flag))

// Memory access helpers
static inline uint8_t cpu_read(Cpu *cpu, uint16_t addr) {
    return cpu->read(cpu->userdata, addr);
}

static inline void cpu_write(Cpu *cpu, uint16_t addr, uint8_t value) {
    cpu->write(cpu->userdata, addr, value);
}

Cpu *cpu_create(void *userdata) {
    Cpu *cpu = (Cpu *)calloc(1, sizeof(Cpu));
    if (!cpu) return NULL;
    
    cpu->userdata = userdata;
    cpu_reset(cpu);
    return cpu;
}

void cpu_destroy(Cpu *cpu) {
    if (cpu) {
        free(cpu);
    }
}

void cpu_reset(Cpu *cpu) {
    // Reset registers to initial state
    cpu->a = 0;
    cpu->x = 0;
    cpu->y = 0;
    cpu->s = 0xFD;  // Stack starts at 0x01FD
    cpu->p = 0x24;  // Bit 5 is always set, bit 4 is unused
    
    // Read reset vector
    uint16_t reset_vector = cpu_read(cpu, 0xFFFC) | (cpu_read(cpu, 0xFFFD) << 8);
    cpu->pc = reset_vector;
    
    // Reset internal state
    cpu->nmi_pending = false;
    cpu->irq_pending = false;
    cpu->cycles = 0;
}

// Stack operations
void cpu_push8(Cpu *cpu, uint8_t value) {
    cpu_write(cpu, 0x0100 + cpu->s, value);
    cpu->s--;
}

uint8_t cpu_pull8(Cpu *cpu) {
    cpu->s++;
    return cpu_read(cpu, 0x0100 + cpu->s);
}

void cpu_push16(Cpu *cpu, uint16_t value) {
    cpu_push8(cpu, (value >> 8) & 0xFF);  // High byte
    cpu_push8(cpu, value & 0xFF);         // Low byte
}

uint16_t cpu_pull16(Cpu *cpu) {
    uint8_t lo = cpu_pull8(cpu);
    uint8_t hi = cpu_pull8(cpu);
    return (hi << 8) | lo;
}

// Flag operations
void cpu_set_flag(Cpu *cpu, uint8_t flag, bool set) {
    if (set) {
        cpu->p |= flag;
    } else {
        cpu->p &= ~flag;
    }
}

bool cpu_get_flag(Cpu *cpu, uint8_t flag) {
    return (cpu->p & flag) != 0;
}

// Addressing modes
uint16_t cpu_addressing_immediate(Cpu *cpu) {
    return cpu->pc++;
}

uint16_t cpu_addressing_zero_page(Cpu *cpu) {
    return cpu_read(cpu, cpu->pc++);
}

uint16_t cpu_addressing_zero_page_x(Cpu *cpu) {
    uint8_t addr = cpu_read(cpu, cpu->pc++);
    return (addr + cpu->x) & 0xFF;
}

uint16_t cpu_addressing_zero_page_y(Cpu *cpu) {
    uint8_t addr = cpu_read(cpu, cpu->pc++);
    return (addr + cpu->y) & 0xFF;
}

uint16_t cpu_addressing_absolute(Cpu *cpu) {
    uint16_t addr = cpu_read(cpu, cpu->pc++);
    addr |= cpu_read(cpu, cpu->pc++) << 8;
    return addr;
}

uint16_t cpu_addressing_absolute_x(Cpu *cpu, bool *page_crossed) {
    uint16_t base = cpu_read(cpu, cpu->pc++);
    base |= cpu_read(cpu, cpu->pc++) << 8;
    
    uint16_t addr = base + cpu->x;
    if (page_crossed) {
        *page_crossed = ((base & 0xFF00) != (addr & 0xFF00));
    }
    
    return addr;
}

uint16_t cpu_addressing_absolute_y(Cpu *cpu, bool *page_crossed) {
    uint16_t base = cpu_read(cpu, cpu->pc++);
    base |= cpu_read(cpu, cpu->pc++) << 8;
    
    uint16_t addr = base + cpu->y;
    if (page_crossed) {
        *page_crossed = ((base & 0xFF00) != (addr & 0xFF00));
    }
    
    return addr;
}

uint16_t cpu_addressing_indirect(Cpu *cpu) {
    uint16_t addr = cpu_read(cpu, cpu->pc++);
    addr |= cpu_read(cpu, cpu->pc++) << 8;
    
    // 6502 has a bug in indirect addressing that doesn't carry to the high byte
    uint16_t indirect_addr = cpu_read(cpu, addr);
    uint16_t next_addr = (addr & 0xFF00) | ((addr + 1) & 0x00FF);
    indirect_addr |= cpu_read(cpu, next_addr) << 8;
    
    return indirect_addr;
}

uint16_t cpu_addressing_indirect_x(Cpu *cpu) {
    uint8_t base = cpu_read(cpu, cpu->pc++);
    uint8_t addr = base + cpu->x;
    
    // Zero-page wrap around
    uint16_t indirect_addr = cpu_read(cpu, addr & 0xFF);
    indirect_addr |= cpu_read(cpu, (addr + 1) & 0xFF) << 8;
    
    return indirect_addr;
}

uint16_t cpu_addressing_indirect_y(Cpu *cpu, bool *page_crossed) {
    uint8_t base = cpu_read(cpu, cpu->pc++);
    
    uint16_t indirect_addr = cpu_read(cpu, base);
    indirect_addr |= cpu_read(cpu, (base + 1) & 0xFF) << 8;
    
    uint16_t addr = indirect_addr + cpu->y;
    if (page_crossed) {
        *page_crossed = ((indirect_addr & 0xFF00) != (addr & 0xFF00));
    }
    
    return addr;
}

// Instruction implementations
void cpu_adc(Cpu *cpu, uint8_t value) {
    uint16_t sum = cpu->a + value + (cpu_get_flag(cpu, CARRY_FLAG) ? 1 : 0);
    
    // Set flags
    cpu_set_flag(cpu, CARRY_FLAG, sum > 0xFF);
    cpu_set_flag(cpu, ZERO_FLAG, (sum & 0xFF) == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, sum & 0x80);
    
    // Overflow flag: If the sign of both inputs is the same and different from the result
    bool overflow = (~(cpu->a ^ value) & (cpu->a ^ sum) & 0x80) != 0;
    cpu_set_flag(cpu, OVERFLOW_FLAG, overflow);
    
    cpu->a = sum & 0xFF;
}

void cpu_and(Cpu *cpu, uint8_t value) {
    cpu->a &= value;
    cpu_set_flag(cpu, ZERO_FLAG, cpu->a == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->a & 0x80);
}

uint8_t cpu_asl(Cpu *cpu, uint8_t value) {
    cpu_set_flag(cpu, CARRY_FLAG, value & 0x80);
    value <<= 1;
    cpu_set_flag(cpu, ZERO_FLAG, value == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, value & 0x80);
    return value;
}

void cpu_bit(Cpu *cpu, uint8_t value) {
    uint8_t result = cpu->a & value;
    cpu_set_flag(cpu, ZERO_FLAG, result == 0);
    cpu_set_flag(cpu, OVERFLOW_FLAG, value & 0x40);
    cpu_set_flag(cpu, NEGATIVE_FLAG, value & 0x80);
}

void cpu_compare(Cpu *cpu, uint8_t reg, uint8_t value) {
    uint8_t result = reg - value;
    cpu_set_flag(cpu, CARRY_FLAG, reg >= value);
    cpu_set_flag(cpu, ZERO_FLAG, reg == value);
    cpu_set_flag(cpu, NEGATIVE_FLAG, result & 0x80);
}

// Instruction implementations (continued)
void cpu_lda(Cpu *cpu, uint8_t value) {
    cpu->a = value;
    cpu_set_flag(cpu, ZERO_FLAG, cpu->a == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->a & 0x80);
}

void cpu_ldx(Cpu *cpu, uint8_t value) {
    cpu->x = value;
    cpu_set_flag(cpu, ZERO_FLAG, cpu->x == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->x & 0x80);
}

void cpu_ldy(Cpu *cpu, uint8_t value) {
    cpu->y = value;
    cpu_set_flag(cpu, ZERO_FLAG, cpu->y == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->y & 0x80);
}

void cpu_sta(Cpu *cpu, uint16_t addr) {
    cpu_write(cpu, addr, cpu->a);
}

void cpu_stx(Cpu *cpu, uint16_t addr) {
    cpu_write(cpu, addr, cpu->x);
}

void cpu_sty(Cpu *cpu, uint16_t addr) {
    cpu_write(cpu, addr, cpu->y);
}

void cpu_tax(Cpu *cpu) {
    cpu->x = cpu->a;
    cpu_set_flag(cpu, ZERO_FLAG, cpu->x == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->x & 0x80);
}

void cpu_tay(Cpu *cpu) {
    cpu->y = cpu->a;
    cpu_set_flag(cpu, ZERO_FLAG, cpu->y == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->y & 0x80);
}

void cpu_txa(Cpu *cpu) {
    cpu->a = cpu->x;
    cpu_set_flag(cpu, ZERO_FLAG, cpu->a == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->a & 0x80);
}

void cpu_tya(Cpu *cpu) {
    cpu->a = cpu->y;
    cpu_set_flag(cpu, ZERO_FLAG, cpu->a == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->a & 0x80);
}

void cpu_tsx(Cpu *cpu) {
    cpu->x = cpu->s;
    cpu_set_flag(cpu, ZERO_FLAG, cpu->x == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->x & 0x80);
}

void cpu_txs(Cpu *cpu) {
    cpu->s = cpu->x;
}

void cpu_pha(Cpu *cpu) {
    cpu_push8(cpu, cpu->a);
}

void cpu_php(Cpu *cpu) {
    cpu_push8(cpu, cpu->p | BREAK_FLAG | UNUSED_FLAG);
}

void cpu_pla(Cpu *cpu) {
    cpu->a = cpu_pull8(cpu);
    cpu_set_flag(cpu, ZERO_FLAG, cpu->a == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->a & 0x80);
}

void cpu_plp(Cpu *cpu) {
    cpu->p = (cpu_pull8(cpu) & ~BREAK_FLAG) | UNUSED_FLAG;
}

// Arithmetic and logical operations
void cpu_eor(Cpu *cpu, uint8_t value) {
    cpu->a ^= value;
    cpu_set_flag(cpu, ZERO_FLAG, cpu->a == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->a & 0x80);
}

void cpu_ora(Cpu *cpu, uint8_t value) {
    cpu->a |= value;
    cpu_set_flag(cpu, ZERO_FLAG, cpu->a == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->a & 0x80);
}

uint8_t cpu_lsr(Cpu *cpu, uint8_t value) {
    cpu_set_flag(cpu, CARRY_FLAG, value & 0x01);
    value >>= 1;
    cpu_set_flag(cpu, ZERO_FLAG, value == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, 0); // LSR always clears the negative flag
    return value;
}

uint8_t cpu_asl(Cpu *cpu, uint8_t value) {
    cpu_set_flag(cpu, CARRY_FLAG, value & 0x80);
    value <<= 1;
    cpu_set_flag(cpu, ZERO_FLAG, value == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, value & 0x80);
    return value;
}

uint8_t cpu_rol(Cpu *cpu, uint8_t value) {
    bool carry = value & 0x80;
    value = (value << 1) | (cpu_get_flag(cpu, CARRY_FLAG) ? 1 : 0);
    cpu_set_flag(cpu, CARRY_FLAG, carry);
    cpu_set_flag(cpu, ZERO_FLAG, value == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, value & 0x80);
    return value;
}

uint8_t cpu_ror(Cpu *cpu, uint8_t value) {
    bool carry = value & 0x01;
    value = (value >> 1) | (cpu_get_flag(cpu, CARRY_FLAG) ? 0x80 : 0);
    cpu_set_flag(cpu, CARRY_FLAG, carry);
    cpu_set_flag(cpu, ZERO_FLAG, value == 0);
    cpu_set_flag(cpu, NEGATIVE_FLAG, value & 0x80);
    return value;
}

// Branch instructions
void cpu_branch(Cpu *cpu, bool condition) {
    if (condition) {
        int8_t offset = (int8_t)cpu_read(cpu, cpu->pc++);
        uint16_t new_pc = cpu->pc + offset;
        
        // Check for page crossing (takes an extra cycle)
        if ((cpu->pc & 0xFF00) != (new_pc & 0xFF00)) {
            cpu->cycles++;
        }
        
        cpu->pc = new_pc;
        cpu->cycles += 2; // Branch taken takes 3 cycles total (1 for opcode, 2 for branch)
    } else {
        cpu->pc++;
        cpu->cycles++; // Branch not taken takes 2 cycles total
    }
}

// Flow control
void cpu_jsr(Cpu *cpu, uint16_t addr) {
    cpu_push16(cpu, cpu->pc - 1);
    cpu->pc = addr;
}

void cpu_rts(Cpu *cpu) {
    cpu->pc = cpu_pull16(cpu) + 1;
}

void cpu_rti(Cpu *cpu) {
    cpu->p = (cpu_pull8(cpu) & ~BREAK_FLAG) | UNUSED_FLAG;
    cpu->pc = cpu_pull16(cpu);
}

// Status flag operations
void cpu_clc(Cpu *cpu) { cpu_set_flag(cpu, CARRY_FLAG, false); }
void cpu_cld(Cpu *cpu) { cpu_set_flag(cpu, DECIMAL_FLAG, false); }
void cpu_cli(Cpu *cpu) { cpu_set_flag(cpu, INTERRUPT_FLAG, false); }
void cpu_clv(Cpu *cpu) { cpu_set_flag(cpu, OVERFLOW_FLAG, false); }
void cpu_sec(Cpu *cpu) { cpu_set_flag(cpu, CARRY_FLAG, true); }
void cpu_sed(Cpu *cpu) { cpu_set_flag(cpu, DECIMAL_FLAG, true); }
void cpu_sei(Cpu *cpu) { cpu_set_flag(cpu, INTERRUPT_FLAG, true); }

// Instruction decoder
static void cpu_execute(Cpu *cpu, uint8_t opcode) {
    switch (opcode) {
        // Load/Store Operations
        case 0xA9: cpu_lda(cpu, cpu_read(cpu, cpu->pc++)); break; // LDA #imm
        case 0xA5: cpu_lda(cpu, cpu_read(cpu, cpu_addressing_zero_page(cpu))); break; // LDA zp
        case 0xB5: cpu_lda(cpu, cpu_read(cpu, cpu_addressing_zero_page_x(cpu))); break; // LDA zp,X
        case 0xAD: cpu_lda(cpu, cpu_read(cpu, cpu_addressing_absolute(cpu))); break; // LDA abs
        case 0xBD: cpu_lda(cpu, cpu_read(cpu, cpu_addressing_absolute_x(cpu, NULL))); break; // LDA abs,X
        case 0xB9: cpu_lda(cpu, cpu_read(cpu, cpu_addressing_absolute_y(cpu, NULL))); break; // LDA abs,Y
        case 0xA1: cpu_lda(cpu, cpu_read(cpu, cpu_addressing_indirect_x(cpu))); break; // LDA (ind,X)
        case 0xB1: { // LDA (ind),Y
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_indirect_y(cpu, &page_crossed);
            if (page_crossed) cpu->cycles++;
            cpu_lda(cpu, cpu_read(cpu, addr));
            break;
        }
        
        // LDX instructions
        case 0xA2: cpu_ldx(cpu, cpu_read(cpu, cpu->pc++)); break; // LDX #imm
        case 0xA6: cpu_ldx(cpu, cpu_read(cpu, cpu_addressing_zero_page(cpu))); break; // LDX zp
        case 0xB6: cpu_ldx(cpu, cpu_read(cpu, cpu_addressing_zero_page_y(cpu))); break; // LDX zp,Y
        case 0xAE: cpu_ldx(cpu, cpu_read(cpu, cpu_addressing_absolute(cpu))); break; // LDX abs
        case 0xBE: { // LDX abs,Y
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_absolute_y(cpu, &page_crossed);
            if (page_crossed) cpu->cycles++;
            cpu_ldx(cpu, cpu_read(cpu, addr));
            break;
        }
        
        // LDY instructions
        case 0xA0: cpu_ldy(cpu, cpu_read(cpu, cpu->pc++)); break; // LDY #imm
        case 0xA4: cpu_ldy(cpu, cpu_read(cpu, cpu_addressing_zero_page(cpu))); break; // LDY zp
        case 0xB4: cpu_ldy(cpu, cpu_read(cpu, cpu_addressing_zero_page_x(cpu))); break; // LDY zp,X
        case 0xAC: cpu_ldy(cpu, cpu_read(cpu, cpu_addressing_absolute(cpu))); break; // LDY abs
        case 0xBC: { // LDY abs,X
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_absolute_x(cpu, &page_crossed);
            if (page_crossed) cpu->cycles++;
            cpu_ldy(cpu, cpu_read(cpu, addr));
            break;
        }
        
        // STA instructions
        case 0x85: cpu_sta(cpu, cpu_addressing_zero_page(cpu)); break; // STA zp
        case 0x95: cpu_sta(cpu, cpu_addressing_zero_page_x(cpu)); break; // STA zp,X
        case 0x8D: cpu_sta(cpu, cpu_addressing_absolute(cpu)); break; // STA abs
        case 0x9D: cpu_sta(cpu, cpu_addressing_absolute_x(cpu, NULL) - cpu->x); break; // STA abs,X (pre-indexed)
        case 0x99: cpu_sta(cpu, cpu_addressing_absolute_y(cpu, NULL) - cpu->y); break; // STA abs,Y (pre-indexed)
        case 0x81: cpu_sta(cpu, cpu_addressing_indirect_x(cpu)); break; // STA (ind,X)
        case 0x91: { // STA (ind),Y
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_indirect_y(cpu, &page_crossed);
            cpu_sta(cpu, addr);
            if (page_crossed) cpu->cycles++;
            break;
        }
        
        // STX/STY instructions
        case 0x86: cpu_stx(cpu, cpu_addressing_zero_page(cpu)); break; // STX zp
        case 0x96: cpu_stx(cpu, cpu_addressing_zero_page_y(cpu)); break; // STX zp,Y
        case 0x8E: cpu_stx(cpu, cpu_addressing_absolute(cpu)); break; // STX abs
        
        case 0x84: cpu_sty(cpu, cpu_addressing_zero_page(cpu)); break; // STY zp
        case 0x94: cpu_sty(cpu, cpu_addressing_zero_page_x(cpu)); break; // STY zp,X
        case 0x8C: cpu_sty(cpu, cpu_addressing_absolute(cpu)); break; // STY abs
        
        // Register transfers
        case 0xAA: cpu_tax(cpu); break; // TAX
        case 0xA8: cpu_tay(cpu); break; // TAY
        case 0x8A: cpu_txa(cpu); break; // TXA
        case 0x98: cpu_tya(cpu); break; // TYA
        case 0x9A: cpu_txs(cpu); break; // TXS
        case 0xBA: cpu_tsx(cpu); break; // TSX
        
        // Stack operations
        case 0x48: cpu_pha(cpu); break; // PHA
        case 0x08: cpu_php(cpu); break; // PHP
        case 0x68: cpu_pla(cpu); break; // PLA
        case 0x28: cpu_plp(cpu); break; // PLP
        
        // Logical operations
        case 0x09: cpu_ora(cpu, cpu_read(cpu, cpu->pc++)); break; // ORA #imm
        case 0x05: cpu_ora(cpu, cpu_read(cpu, cpu_addressing_zero_page(cpu))); break; // ORA zp
        case 0x15: cpu_ora(cpu, cpu_read(cpu, cpu_addressing_zero_page_x(cpu))); break; // ORA zp,X
        case 0x0D: cpu_ora(cpu, cpu_read(cpu, cpu_addressing_absolute(cpu))); break; // ORA abs
        case 0x1D: { // ORA abs,X
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_absolute_x(cpu, &page_crossed);
            if (page_crossed) cpu->cycles++;
            cpu_ora(cpu, cpu_read(cpu, addr));
            break;
        }
        case 0x19: { // ORA abs,Y
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_absolute_y(cpu, &page_crossed);
            if (page_crossed) cpu->cycles++;
            cpu_ora(cpu, cpu_read(cpu, addr));
            break;
        }
        case 0x01: cpu_ora(cpu, cpu_read(cpu, cpu_addressing_indirect_x(cpu))); break; // ORA (ind,X)
        case 0x11: { // ORA (ind),Y
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_indirect_y(cpu, &page_crossed);
            if (page_crossed) cpu->cycles++;
            cpu_ora(cpu, cpu_read(cpu, addr));
            break;
        }
        
        // EOR instructions (similar pattern to ORA)
        case 0x49: cpu_eor(cpu, cpu_read(cpu, cpu->pc++)); break; // EOR #imm
        case 0x45: cpu_eor(cpu, cpu_read(cpu, cpu_addressing_zero_page(cpu))); break; // EOR zp
        case 0x55: cpu_eor(cpu, cpu_read(cpu, cpu_addressing_zero_page_x(cpu))); break; // EOR zp,X
        case 0x4D: cpu_eor(cpu, cpu_read(cpu, cpu_addressing_absolute(cpu))); break; // EOR abs
        case 0x5D: { // EOR abs,X
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_absolute_x(cpu, &page_crossed);
            if (page_crossed) cpu->cycles++;
            cpu_eor(cpu, cpu_read(cpu, addr));
            break;
        }
        case 0x59: { // EOR abs,Y
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_absolute_y(cpu, &page_crossed);
            if (page_crossed) cpu->cycles++;
            cpu_eor(cpu, cpu_read(cpu, addr));
            break;
        }
        case 0x41: cpu_eor(cpu, cpu_read(cpu, cpu_addressing_indirect_x(cpu))); break; // EOR (ind,X)
        case 0x51: { // EOR (ind),Y
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_indirect_y(cpu, &page_crossed);
            if (page_crossed) cpu->cycles++;
            cpu_eor(cpu, cpu_read(cpu, addr));
            break;
        }
        
        // AND instructions (similar pattern to ORA/EOR)
        case 0x29: cpu_and(cpu, cpu_read(cpu, cpu->pc++)); break; // AND #imm
        case 0x25: cpu_and(cpu, cpu_read(cpu, cpu_addressing_zero_page(cpu))); break; // AND zp
        case 0x35: cpu_and(cpu, cpu_read(cpu, cpu_addressing_zero_page_x(cpu))); break; // AND zp,X
        case 0x2D: cpu_and(cpu, cpu_read(cpu, cpu_addressing_absolute(cpu))); break; // AND abs
        case 0x3D: { // AND abs,X
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_absolute_x(cpu, &page_crossed);
            if (page_crossed) cpu->cycles++;
            cpu_and(cpu, cpu_read(cpu, addr));
            break;
        }
        case 0x39: { // AND abs,Y
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_absolute_y(cpu, &page_crossed);
            if (page_crossed) cpu->cycles++;
            cpu_and(cpu, cpu_read(cpu, addr));
            break;
        }
        case 0x21: cpu_and(cpu, cpu_read(cpu, cpu_addressing_indirect_x(cpu))); break; // AND (ind,X)
        case 0x31: { // AND (ind),Y
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_indirect_y(cpu, &page_crossed);
            if (page_crossed) cpu->cycles++;
            cpu_and(cpu, cpu_read(cpu, addr));
            break;
        }
        
        // BIT instruction
        case 0x24: { // BIT zp
            uint16_t addr = cpu_addressing_zero_page(cpu);
            cpu_bit(cpu, cpu_read(cpu, addr));
            break;
        }
        case 0x2C: { // BIT abs
            uint16_t addr = cpu_addressing_absolute(cpu);
            cpu_bit(cpu, cpu_read(cpu, addr));
            break;
        }
        
        // Shift instructions
        case 0x0A: cpu->a = cpu_asl(cpu, cpu->a); break; // ASL A
        case 0x06: { // ASL zp
            uint16_t addr = cpu_addressing_zero_page(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_asl(cpu, value));
            break;
        }
        case 0x16: { // ASL zp,X
            uint16_t addr = cpu_addressing_zero_page_x(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_asl(cpu, value));
            break;
        }
        case 0x0E: { // ASL abs
            uint16_t addr = cpu_addressing_absolute(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_asl(cpu, value));
            break;
        }
        case 0x1E: { // ASL abs,X
            uint16_t addr = cpu_addressing_absolute_x(cpu, NULL);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_asl(cpu, value));
            cpu->cycles++; // Extra cycle for page crossing
            break;
        }
        
        // LSR instructions (similar to ASL)
        case 0x4A: cpu->a = cpu_lsr(cpu, cpu->a); break; // LSR A
        case 0x46: { // LSR zp
            uint16_t addr = cpu_addressing_zero_page(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_lsr(cpu, value));
            break;
        }
        case 0x56: { // LSR zp,X
            uint16_t addr = cpu_addressing_zero_page_x(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_lsr(cpu, value));
            break;
        }
        case 0x4E: { // LSR abs
            uint16_t addr = cpu_addressing_absolute(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_lsr(cpu, value));
            break;
        }
        case 0x5E: { // LSR abs,X
            uint16_t addr = cpu_addressing_absolute_x(cpu, NULL);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_lsr(cpu, value));
            cpu->cycles++; // Extra cycle for page crossing
            break;
        }
        
        // ROL instructions
        case 0x2A: cpu->a = cpu_rol(cpu, cpu->a); break; // ROL A
        case 0x26: { // ROL zp
            uint16_t addr = cpu_addressing_zero_page(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_rol(cpu, value));
            break;
        }
        case 0x36: { // ROL zp,X
            uint16_t addr = cpu_addressing_zero_page_x(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_rol(cpu, value));
            break;
        }
        case 0x2E: { // ROL abs
            uint16_t addr = cpu_addressing_absolute(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_rol(cpu, value));
            break;
        }
        case 0x3E: { // ROL abs,X
            uint16_t addr = cpu_addressing_absolute_x(cpu, NULL);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_rol(cpu, value));
            cpu->cycles++; // Extra cycle for page crossing
            break;
        }
        
        // ROR instructions
        case 0x6A: cpu->a = cpu_ror(cpu, cpu->a); break; // ROR A
        case 0x66: { // ROR zp
            uint16_t addr = cpu_addressing_zero_page(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_ror(cpu, value));
            break;
        }
        case 0x76: { // ROR zp,X
            uint16_t addr = cpu_addressing_zero_page_x(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_ror(cpu, value));
            break;
        }
        case 0x6E: { // ROR abs
            uint16_t addr = cpu_addressing_absolute(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_ror(cpu, value));
            break;
        }
        case 0x7E: { // ROR abs,X
            uint16_t addr = cpu_addressing_absolute_x(cpu, NULL);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, cpu_ror(cpu, value));
            cpu->cycles++; // Extra cycle for page crossing
            break;
        }
        
        // Branch instructions
        case 0x10: cpu_branch(cpu, !cpu_get_flag(cpu, NEGATIVE_FLAG)); break; // BPL
        case 0x30: cpu_branch(cpu, cpu_get_flag(cpu, NEGATIVE_FLAG)); break;  // BMI
        case 0x50: cpu_branch(cpu, !cpu_get_flag(cpu, OVERFLOW_FLAG)); break; // BVC
        case 0x70: cpu_branch(cpu, cpu_get_flag(cpu, OVERFLOW_FLAG)); break;  // BVS
        case 0x90: cpu_branch(cpu, !cpu_get_flag(cpu, CARRY_FLAG)); break;    // BCC
        case 0xB0: cpu_branch(cpu, cpu_get_flag(cpu, CARRY_FLAG)); break;     // BCS
        case 0xD0: cpu_branch(cpu, !cpu_get_flag(cpu, ZERO_FLAG)); break;     // BNE
        case 0xF0: cpu_branch(cpu, cpu_get_flag(cpu, ZERO_FLAG)); break;      // BEQ
        
        // Status flag operations
        case 0x18: cpu_clc(cpu); break; // CLC
        case 0xD8: cpu_cld(cpu); break; // CLD
        case 0x58: cpu_cli(cpu); break; // CLI
        case 0xB8: cpu_clv(cpu); break; // CLV
        case 0x38: cpu_sec(cpu); break; // SEC
        case 0xF8: cpu_sed(cpu); break; // SED
        case 0x78: cpu_sei(cpu); break; // SEI
        
        // Compare instructions
        case 0xC9: cpu_compare(cpu, cpu->a, cpu_read(cpu, cpu->pc++)); break; // CMP #imm
        case 0xC5: cpu_compare(cpu, cpu->a, cpu_read(cpu, cpu_addressing_zero_page(cpu))); break; // CMP zp
        case 0xD5: cpu_compare(cpu, cpu->a, cpu_read(cpu, cpu_addressing_zero_page_x(cpu))); break; // CMP zp,X
        case 0xCD: cpu_compare(cpu, cpu->a, cpu_read(cpu, cpu_addressing_absolute(cpu))); break; // CMP abs
        case 0xDD: { // CMP abs,X
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_absolute_x(cpu, &page_crossed);
            if (page_crossed) cpu->cycles++;
            cpu_compare(cpu, cpu->a, cpu_read(cpu, addr));
            break;
        }
        case 0xD9: { // CMP abs,Y
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_absolute_y(cpu, &page_crossed);
            if (page_crossed) cpu->cycles++;
            cpu_compare(cpu, cpu->a, cpu_read(cpu, addr));
            break;
        }
        case 0xC1: cpu_compare(cpu, cpu->a, cpu_read(cpu, cpu_addressing_indirect_x(cpu))); break; // CMP (ind,X)
        case 0xD1: { // CMP (ind),Y
            bool page_crossed = false;
            uint16_t addr = cpu_addressing_indirect_y(cpu, &page_crossed);
            if (page_crossed) cpu->cycles++;
            cpu_compare(cpu, cpu->a, cpu_read(cpu, addr));
            break;
        }
        
        // CPX/CPY instructions (similar to CMP)
        case 0xE0: cpu_compare(cpu, cpu->x, cpu_read(cpu, cpu->pc++)); break; // CPX #imm
        case 0xE4: cpu_compare(cpu, cpu->x, cpu_read(cpu, cpu_addressing_zero_page(cpu))); break; // CPX zp
        case 0xEC: cpu_compare(cpu, cpu->x, cpu_read(cpu, cpu_addressing_absolute(cpu))); break; // CPX abs
        
        case 0xC0: cpu_compare(cpu, cpu->y, cpu_read(cpu, cpu->pc++)); break; // CPY #imm
        case 0xC4: cpu_compare(cpu, cpu->y, cpu_read(cpu, cpu_addressing_zero_page(cpu))); break; // CPY zp
        case 0xCC: cpu_compare(cpu, cpu->y, cpu_read(cpu, cpu_addressing_absolute(cpu))); break; // CPY abs
        
        // Increment/Decrement
        case 0xE6: { // INC zp
            uint16_t addr = cpu_addressing_zero_page(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, value + 1);
            cpu_set_flag(cpu, ZERO_FLAG, (value + 1) == 0);
            cpu_set_flag(cpu, NEGATIVE_FLAG, (value + 1) & 0x80);
            break;
        }
        case 0xF6: { // INC zp,X
            uint16_t addr = cpu_addressing_zero_page_x(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, value + 1);
            cpu_set_flag(cpu, ZERO_FLAG, (value + 1) == 0);
            cpu_set_flag(cpu, NEGATIVE_FLAG, (value + 1) & 0x80);
            break;
        }
        case 0xEE: { // INC abs
            uint16_t addr = cpu_addressing_absolute(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, value + 1);
            cpu_set_flag(cpu, ZERO_FLAG, (value + 1) == 0);
            cpu_set_flag(cpu, NEGATIVE_FLAG, (value + 1) & 0x80);
            break;
        }
        case 0xFE: { // INC abs,X
            uint16_t addr = cpu_addressing_absolute_x(cpu, NULL);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, value + 1);
            cpu_set_flag(cpu, ZERO_FLAG, (value + 1) == 0);
            cpu_set_flag(cpu, NEGATIVE_FLAG, (value + 1) & 0x80);
            cpu->cycles++; // Extra cycle for page crossing
            break;
        }
        
        // DEC instructions (similar to INC)
        case 0xC6: { // DEC zp
            uint16_t addr = cpu_addressing_zero_page(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, value - 1);
            cpu_set_flag(cpu, ZERO_FLAG, (value - 1) == 0);
            cpu_set_flag(cpu, NEGATIVE_FLAG, (value - 1) & 0x80);
            break;
        }
        case 0xD6: { // DEC zp,X
            uint16_t addr = cpu_addressing_zero_page_x(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, value - 1);
            cpu_set_flag(cpu, ZERO_FLAG, (value - 1) == 0);
            cpu_set_flag(cpu, NEGATIVE_FLAG, (value - 1) & 0x80);
            break;
        }
        case 0xCE: { // DEC abs
            uint16_t addr = cpu_addressing_absolute(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, value - 1);
            cpu_set_flag(cpu, ZERO_FLAG, (value - 1) == 0);
            cpu_set_flag(cpu, NEGATIVE_FLAG, (value - 1) & 0x80);
            break;
        }
        case 0xDE: { // DEC abs,X
            uint16_t addr = cpu_addressing_absolute_x(cpu, NULL);
            uint8_t value = cpu_read(cpu, addr);
            cpu_write(cpu, addr, value - 1);
            cpu_set_flag(cpu, ZERO_FLAG, (value - 1) == 0);
            cpu_set_flag(cpu, NEGATIVE_FLAG, (value - 1) & 0x80);
            cpu->cycles++; // Extra cycle for page crossing
            break;
        }
        
        // Increment/Decrement registers
        case 0xE8: // INX
            cpu->x++;
            cpu_set_flag(cpu, ZERO_FLAG, cpu->x == 0);
            cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->x & 0x80);
            break;
            
        case 0xC8: // INY
            cpu->y++;
            cpu_set_flag(cpu, ZERO_FLAG, cpu->y == 0);
            cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->y & 0x80);
            break;
            
        case 0xCA: // DEX
            cpu->x--;
            cpu_set_flag(cpu, ZERO_FLAG, cpu->x == 0);
            cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->x & 0x80);
            break;
            
        case 0x88: // DEY
            cpu->y--;
            cpu_set_flag(cpu, ZERO_FLAG, cpu->y == 0);
            cpu_set_flag(cpu, NEGATIVE_FLAG, cpu->y & 0x80);
            break;
        
        // Jumps and subroutines
        case 0x4C: // JMP abs
            cpu->pc = cpu_addressing_absolute(cpu);
            break;
            
        case 0x6C: { // JMP (ind)
            uint16_t addr = cpu_addressing_absolute(cpu);
            // Handle 6502 page boundary bug
            uint16_t indirect_addr = cpu_read(cpu, addr);
            if ((addr & 0x00FF) == 0x00FF) {
                // Simulate the 6502 page boundary bug
                indirect_addr |= (cpu_read(cpu, addr & 0xFF00) << 8);
            } else {
                indirect_addr |= (cpu_read(cpu, addr + 1) << 8);
            }
            cpu->pc = indirect_addr;
            break;
        }
        
        case 0x20: { // JSR
            uint16_t target = cpu_addressing_absolute(cpu);
            cpu_push16(cpu, cpu->pc - 1);
            cpu->pc = target;
            break;
        }
        
        case 0x60: // RTS
            cpu->pc = cpu_pull16(cpu) + 1;
            break;
        
        // Interrupt handling
        case 0x00: // BRK
            cpu_push16(cpu, cpu->pc + 1);
            cpu_php(cpu);
            cpu_sei(cpu);
            cpu->pc = cpu_read16(cpu, 0xFFFE);
            break;
            
        case 0x40: // RTI
            cpu_plp(cpu);
            cpu->pc = cpu_pull16(cpu);
            break;
            
        // No operation
        case 0xEA: // NOP
            break;
        
        // Unofficial/illegal opcodes
        case 0x1A: case 0x3A: case 0x5A: case 0x7A: case 0xDA: case 0xFA:
            // NOP (unofficial 1-byte NOPs)
            break;
            
        default:
            // Handle unknown opcode (could implement as NOP or crash)
            // For now, we'll just log and continue
            fprintf(stderr, "Unknown opcode: 0x%02X at 0x%04X\n", opcode, cpu->pc - 1);
            break;
    }
}

// Main CPU execution loop
int cpu_step(Cpu *cpu) {
    if (cpu->nmi_pending) {
        cpu_nmi(cpu);
        return 7;  // NMI takes 7 cycles
    }
    
    if (cpu->irq_pending && !cpu_get_flag(cpu, INTERRUPT_FLAG)) {
        cpu_irq(cpu);
        return 7;  // IRQ takes 7 cycles
    }
    
    uint8_t opcode = cpu_read(cpu, cpu->pc++);
    int cycles = 0;
    
    // Decode and execute instruction
    switch (opcode) {
        // LDA (Load Accumulator)
        case 0xA9: {  // Immediate
            uint8_t value = cpu_read(cpu, cpu->pc++);
            cpu_lda(cpu, value);
            cycles = 2;
            break;
        }
        case 0xA5: {  // Zero Page
            uint16_t addr = cpu_addressing_zero_page(cpu);
            uint8_t value = cpu_read(cpu, addr);
            cpu_lda(cpu, value);
            cycles = 3;
            break;
        }
        // ... (more opcode cases would follow)
        
        default:
            // Handle unknown opcode (could implement as NOP or crash)
            fprintf(stderr, "Unknown opcode: 0x%02X at 0x%04X\n", opcode, cpu->pc - 1);
            cycles = 1;
            break;
    }
    
    cpu->cycles += cycles;
    return cycles;
}

// Interrupt handling
void cpu_nmi(Cpu *cpu) {
    cpu_push16(cpu, cpu->pc);
    cpu_push8(cpu, cpu->p | 0x20);  // Push with Break flag set
    cpu_set_flag(cpu, INTERRUPT_FLAG, true);
    
    // Jump to NMI vector
    cpu->pc = cpu_read(cpu, 0xFFFA) | (cpu_read(cpu, 0xFFFB) << 8);
    
    cpu->nmi_pending = false;
}

void cpu_irq(Cpu *cpu) {
    cpu_push16(cpu, cpu->pc);
    cpu_push8(cpu, cpu->p | 0x20);  // Push with Break flag set
    cpu_set_flag(cpu, INTERRUPT_FLAG, true);
    
    // Jump to IRQ/BRK vector
    cpu->pc = cpu_read(cpu, 0xFFFE) | (cpu_read(cpu, 0xFFFF) << 8);
    
    cpu->irq_pending = false;
}
