#ifndef EMULATOR_H
#define EMULATOR_H

#include <gtk/gtk.h>

// Initialize the emulator with a GTK drawing area
void emulator_init(GtkWidget *drawing_area);

// Load a ROM file into the emulator
gboolean emulator_load_rom(const char *filename);

// Start the emulation
void emulator_start(void);

// Stop the emulation
void emulator_stop(void);

// Reset the emulator
void emulator_reset(void);

// Clean up resources
void emulator_cleanup(void);

#endif // EMULATOR_H
