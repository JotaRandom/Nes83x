# Nes83x - NES Emulator

A minimal NES emulator written in C with GTK4 for the user interface, inspired by Snes9x.

## Features

- [ ] CPU emulation (6502)
- [ ] PPU emulation (Picture Processing Unit)
- [ ] Basic input handling
- [ ] ROM loading
- [ ] Basic video output
- [ ] Sound (stretch goal)

## Dependencies

### Arch Linux (WSL)

```bash
sudo pacman -S base-devel meson ninja gtk4 sdl3
```

## Building

1. Clone the repository
2. Create a build directory and configure with Meson:

```bash
meson setup build
```

3. Build the project:

```bash
cd build
ninja
```

4. Run the emulator:

```bash
./nes83x
```

## Running ROMs

```bash
./nes83x /path/to/rom.nes
```

## Controls

- **Arrow Keys**: D-Pad
- **Z**: A Button
- **X**: B Button
- **Enter**: Start
- **Shift**: Select
- **Escape**: Quit

## License

MIT License
