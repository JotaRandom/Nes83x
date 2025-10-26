#include "emulator.h"
#include "nes.h"
#include <stdio.h>

static GtkWidget *g_drawing_area = NULL;
static Nes *g_nes = NULL;

void emulator_init(GtkWidget *drawing_area) {
    g_return_if_fail(GTK_IS_DRAWING_AREA(drawing_area));
    
    g_drawing_area = drawing_area;
    
    // Initialize NES
    g_nes = nes_create();
    if (!g_nes) {
        g_critical("Failed to initialize NES emulator");
        return;
    }
    
    // Set up drawing
    gtk_drawing_area_set_draw_func(
        GTK_DRAWING_AREA(drawing_area),
        emulator_draw_cb,
        NULL,
        NULL
    );
    
    // Set up input handling
    gtk_widget_add_controller(drawing_area, gtk_event_controller_key_new());
    g_signal_connect(drawing_area, "key-pressed", G_CALLBACK(emulator_key_pressed), NULL);
    g_signal_connect(drawing_area, "key-released", G_CALLBACK(emulator_key_released), NULL);
}

gboolean emulator_load_rom(const char *filename) {
    g_return_val_if_fail(g_nes != NULL, FALSE);
    g_return_val_if_fail(filename != NULL, FALSE);
    
    return nes_load_rom(g_nes, filename);
}

void emulator_start(void) {
    g_return_if_fail(g_nes != NULL);
    // TODO: Start emulation loop
}

void emulator_stop(void) {
    g_return_if_fail(g_nes != NULL);
    // TODO: Stop emulation loop
}

void emulator_reset(void) {
    g_return_if_fail(g_nes != NULL);
    nes_reset(g_nes);
}

void emulator_cleanup(void) {
    if (g_nes) {
        nes_destroy(g_nes);
        g_nes = NULL;
    }
}

static void emulator_draw_cb(GtkDrawingArea *drawing_area, cairo_t *cr, int width, int height, gpointer data) {
    // Set black background
    cairo_set_source_rgb(cr, 0, 0, 0);
    cairo_paint(cr);
    
    if (!g_nes) {
        return;
    }
    
    // TODO: Draw NES frame buffer
}

static gboolean emulator_key_pressed(GtkEventControllerKey *controller, guint keyval, guint keycode, GdkModifierType state, gpointer user_data) {
    // TODO: Handle key press
    return FALSE;
}

static gboolean emulator_key_released(GtkEventControllerKey *controller, guint keyval, guint keycode, GdkModifierType state, gpointer user_data) {
    // TODO: Handle key release
    return FALSE;
}
