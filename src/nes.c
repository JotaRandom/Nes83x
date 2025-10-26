#include "nes.h"
#include "cpu.h"
#include "ppu.h"
#include "rom.h"
#include <stdlib.h>
#include <string.h>

Nes *nes_create(void) {
    Nes *nes = (Nes *)malloc(sizeof(Nes));
    if (!nes) {
        return NULL;
    }
    
    memset(nes, 0, sizeof(Nes));
    
    // Initialize CPU and PPU
    nes->cpu = cpu_create(nes);
    nes->ppu = ppu_create(nes);
    
    if (!nes->cpu || !nes->ppu) {
        if (nes->cpu) cpu_destroy(nes->cpu);
        if (nes->ppu) ppu_destroy(nes->ppu);
        free(nes);
        return NULL;
    }
    
    return nes;
}

void nes_destroy(Nes *nes) {
    if (!nes) return;
    
    if (nes->cpu) cpu_destroy(nes->cpu);
    if (nes->ppu) ppu_destroy(nes->ppu);
    if (nes->rom) rom_destroy(nes->rom);
    
    free(nes);
}

void nes_reset(Nes *nes) {
    if (!nes) return;
    
    // Reset CPU and PPU
    cpu_reset(nes->cpu);
    ppu_reset(nes->ppu);
    
    // Clear RAM
    memset(nes->ram, 0, sizeof(nes->ram));
    
    // Reset controllers
    nes->controller1 = 0;
    nes->controller2 = 0;
    
    nes->running = true;
}

bool nes_load_rom(Nes *nes, const char *filename) {
    if (!nes || !filename) return false;
    
    // Free existing ROM if any
    if (nes->rom) {
        rom_destroy(nes->rom);
        nes->rom = NULL;
    }
    
    // Load new ROM
    nes->rom = rom_load(filename);
    if (!nes->rom) {
        return false;
    }
    
    // Reset the system after loading ROM
    nes_reset(nes);
    return true;
}

void nes_step(Nes *nes) {
    if (!nes || !nes->running) return;
    
    // Execute one CPU instruction
    cpu_step(nes->cpu);
    
    // Run PPU for the same number of cycles
    // (3 PPU cycles per CPU cycle)
    for (int i = 0; i < 3; i++) {
        ppu_step(nes->ppu);
    }
}

void nes_frame(Nes *nes) {
    if (!nes || !nes->running) return;
    
    // Run until a full frame is rendered
    // This is a simplified version - a real emulator would be more precise
    for (int i = 0; i < 29781; i++) {  // Approximate number of CPU cycles per frame
        nes_step(nes);
    }
}

uint8_t nes_cpu_read(Nes *nes, uint16_t addr) {
    if (!nes) return 0;
    
    // Handle RAM (0x0000-0x1FFF, mirrored every 0x0800)
    if (addr < 0x2000) {
        return nes->ram[addr & 0x07FF];
    }
    
    // PPU registers (0x2000-0x3FFF, mirrored every 8 bytes)
    else if (addr < 0x4000) {
        return ppu_register_read(nes->ppu, 0x2000 + (addr & 0x0007));
    }
    
    // APU and I/O registers (simplified)
    else if (addr == 0x4016) {
        // Controller 1
        uint8_t ret = (nes->controller1 & 0x80) ? 1 : 0;
        nes->controller1 <<= 1;
        return ret;
    }
    
    // ROM (0x8000-0xFFFF)
    else if (addr >= 0x8000) {
        if (nes->rom) {
            return rom_read_prg(nes->rom, addr - 0x8000);
        }
    }
    
    return 0;
}

void nes_cpu_write(Nes *nes, uint16_t addr, uint8_t value) {
    if (!nes) return;
    
    // Handle RAM (0x0000-0x1FFF, mirrored every 0x0800)
    if (addr < 0x2000) {
        nes->ram[addr & 0x07FF] = value;
    }
    
    // PPU registers (0x2000-0x3FFF, mirrored every 8 bytes)
    else if (addr < 0x4000) {
        ppu_register_write(nes->ppu, 0x2000 + (addr & 0x0007), value);
    }
    
    // APU and I/O registers (simplified)
    else if (addr == 0x4016) {
        // Controller latch
        if (value & 0x01) {
            // Reload shift registers
            // In a real emulator, you would read the actual controller state here
        }
    }
}

uint8_t nes_ppu_read(Nes *nes, uint16_t addr) {
    if (!nes || !nes->rom) return 0;
    
    // Pattern tables (0x0000-0x1FFF)
    if (addr < 0x2000) {
        return rom_read_chr(nes->rom, addr);
    }
    
    // Name tables (0x2000-0x2FFF)
    else if (addr < 0x3000) {
        // Simplified: just return 0 for now
        return 0;
    }
    
    // Mirrors of 0x2000-0x2EFF (0x3000-0x3EFF)
    else if (addr < 0x3F00) {
        return nes_ppu_read(nes, addr - 0x1000);
    }
    
    // Palette RAM indices (0x3F00-0x3FFF)
    else {
        // Simplified: just return 0 for now
        return 0;
    }
}

void nes_ppu_write(Nes *nes, uint16_t addr, uint8_t value) {
    if (!nes) return;
    
    // Pattern tables (0x0000-0x1FFF) - typically read-only, but some mappers allow writing
    if (addr < 0x2000) {
        // Do nothing for now
    }
    
    // Name tables (0x2000-0x2FFF)
    else if (addr < 0x3000) {
        // Simplified: do nothing for now
    }
    
    // Mirrors of 0x2000-0x2EFF (0x3000-0x3EFF)
    else if (addr < 0x3F00) {
        nes_ppu_write(nes, addr - 0x1000, value);
    }
    
    // Palette RAM indices (0x3F00-0x3FFF)
    else {
        // Simplified: do nothing for now
    }
}
