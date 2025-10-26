#ifndef NES_H
#define NES_H

#include <stdint.h>
#include <stdbool.h>

typedef struct Cpu Cpu;
typedef struct Ppu Ppu;
typedef struct Rom Rom;

typedef struct {
    Cpu *cpu;
    Ppu *ppu;
    Rom *rom;
    
    // Memory (2KB internal RAM + mirrors)
    uint8_t ram[0x0800];
    
    // Controller states
    uint8_t controller1;
    uint8_t controller2;
    
    bool running;
} Nes;

// Create a new NES instance
Nes *nes_create(void);

// Destroy an NES instance
void nes_destroy(Nes *nes);

// Reset the NES
void nes_reset(Nes *nes);

// Load a ROM file
bool nes_load_rom(Nes *nes, const char *filename);

// Run a single CPU instruction
void nes_step(Nes *nes);

// Run a single frame
void nes_frame(Nes *nes);

// Read from CPU memory
uint8_t nes_cpu_read(Nes *nes, uint16_t addr);

// Write to CPU memory
void nes_cpu_write(Nes *nes, uint16_t addr, uint8_t value);

// Read from PPU memory
uint8_t nes_ppu_read(Nes *nes, uint16_t addr);

// Write to PPU memory
void nes_ppu_write(Nes *nes, uint16_t addr, uint8_t value);

#endif // NES_H
