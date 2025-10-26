#ifndef CPU_H
#define CPU_H

#include <stdint.h>
#include <stdbool.h>

// Status register flags
#define CARRY_FLAG      0x01
#define ZERO_FLAG       0x02
#define INTERRUPT_FLAG  0x04
#define DECIMAL_FLAG    0x08
#define BREAK_FLAG      0x10
#define UNUSED_FLAG     0x20
#define OVERFLOW_FLAG   0x40
#define NEGATIVE_FLAG   0x80

typedef struct Cpu {
    // Registers
    uint8_t a;      // Accumulator
    uint8_t x;      // X index register
    uint8_t y;      // Y index register
    uint8_t s;      // Stack pointer (points to location on bus)
    uint16_t pc;    // Program counter
    uint8_t p;      // Status register
    
    // Memory interface
    void *userdata; // Pointer to NES struct
    
    // Internal state
    bool nmi_pending;
    bool irq_pending;
    int cycles;     // Cycle counter for timing
    
    // Callback for memory access
    uint8_t (*read)(void *userdata, uint16_t addr);
    void (*write)(void *userdata, uint16_t addr, uint8_t value);
} Cpu;

// Create a new CPU instance
Cpu *cpu_create(void *userdata);

// Destroy a CPU instance
void cpu_destroy(Cpu *cpu);

// Reset the CPU
void cpu_reset(Cpu *cpu);

// Execute a single CPU instruction
int cpu_step(Cpu *cpu);

// Trigger a non-maskable interrupt
void cpu_nmi(Cpu *cpu);

// Trigger an interrupt request
void cpu_irq(Cpu *cpu);

// Helper functions for addressing modes
uint16_t cpu_addressing_immediate(Cpu *cpu);
uint16_t cpu_addressing_zero_page(Cpu *cpu);
uint16_t cpu_addressing_zero_page_x(Cpu *cpu);
uint16_t cpu_addressing_zero_page_y(Cpu *cpu);
uint16_t cpu_addressing_absolute(Cpu *cpu);
uint16_t cpu_addressing_absolute_x(Cpu *cpu, bool *page_crossed);
uint16_t cpu_addressing_absolute_y(Cpu *cpu, bool *page_crossed);
uint16_t cpu_addressing_indirect(Cpu *cpu);
uint16_t cpu_addressing_indirect_x(Cpu *cpu);
uint16_t cpu_addressing_indirect_y(Cpu *cpu, bool *page_crossed);

// Helper functions for instructions
void cpu_push8(Cpu *cpu, uint8_t value);
uint8_t cpu_pull8(Cpu *cpu);
void cpu_push16(Cpu *cpu, uint16_t value);
uint16_t cpu_pull16(Cpu *cpu);
void cpu_set_flag(Cpu *cpu, uint8_t flag, bool set);
bool cpu_get_flag(Cpu *cpu, uint8_t flag);

// Instruction implementations
void cpu_adc(Cpu *cpu, uint8_t value);
void cpu_and(Cpu *cpu, uint8_t value);
uint8_t cpu_asl(Cpu *cpu, uint8_t value);
void cpu_bit(Cpu *cpu, uint8_t value);
void cpu_compare(Cpu *cpu, uint8_t reg, uint8_t value);
void cpu_dec(Cpu *cpu, uint16_t addr);
void cpu_dex(Cpu *cpu);
void cpu_dey(Cpu *cpu);
void cpu_eor(Cpu *cpu, uint8_t value);
void cpu_inc(Cpu *cpu, uint16_t addr);
void cpu_inx(Cpu *cpu);
void cpu_iny(Cpu *cpu);
void cpu_jsr(Cpu *cpu, uint16_t addr);
void cpu_lda(Cpu *cpu, uint8_t value);
void cpu_ldx(Cpu *cpu, uint8_t value);
void cpu_ldy(Cpu *cpu, uint8_t value);
uint8_t cpu_lsr(Cpu *cpu, uint8_t value);
void cpu_ora(Cpu *cpu, uint8_t value);
void cpu_pha(Cpu *cpu);
void cpu_php(Cpu *cpu);
void cpu_pla(Cpu *cpu);
void cpu_plp(Cpu *cpu);
uint8_t cpu_rol(Cpu *cpu, uint8_t value);
uint8_t cpu_ror(Cpu *cpu, uint8_t value);
void cpu_rti(Cpu *cpu);
void cpu_rts(Cpu *cpu);
void cpu_sbc(Cpu *cpu, uint8_t value);
void cpu_sta(Cpu *cpu, uint16_t addr);
void cpu_stx(Cpu *cpu, uint16_t addr);
void cpu_sty(Cpu *cpu, uint16_t addr);
void cpu_tax(Cpu *cpu);
void cpu_tay(Cpu *cpu);
void cpu_tsx(Cpu *cpu);
void cpu_txa(Cpu *cpu);
void cpu_txs(Cpu *cpu);
void cpu_tya(Cpu *cpu);

// Unofficial/illegal opcodes (simplified)
void cpu_slo(Cpu *cpu, uint16_t addr);
void cpu_rla(Cpu *cpu, uint16_t addr);
void cpu_sre(Cpu *cpu, uint16_t addr);
void cpu_rra(Cpu *cpu, uint16_t addr);
void cpu_sax(Cpu *cpu, uint16_t addr);
void cpu_lax(Cpu *cpu, uint16_t addr);
void cpu_dcp(Cpu *cpu, uint16_t addr);
void cpu_isc(Cpu *cpu, uint16_t addr);

#endif // CPU_H
