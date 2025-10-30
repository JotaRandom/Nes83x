# NES83x - NES Emulator in Rust

A NES emulator written in Rust with GTK4 for the user interface.

[Experimenting with the use of AI to create it entirely]

## Features

### CPU (6502) Implementation Status
- [x] All official 6502 instructions
- [x] Cycle-accurate timing
- [x] Interrupt handling (NMI, IRQ, BRK)
- [x] Status flags and processor state
- [x] Unofficial/illegal opcodes (partially implemented)
- [x] Memory-mapped I/O stubs

### PPU (Picture Processing Unit) Implementation Status
- [x] Background rendering with scrolling
- [x] Sprite rendering with priority
- [x] Sprite zero hit detection
- [x] NMI generation
- [x] VRAM/CRAM memory management
- [x] Support for different mirroring modes

### APU (Audio Processing Unit) Implementation Status
- [x] Complete APU implementation with accurate timing
- [x] Two pulse wave channels with configurable duty cycles
  - [x] Sweep unit for frequency modulation
  - [x] Volume envelope and length counter
- [x] Triangle wave channel
  - [x] Linear counter for waveform generation
  - [x] Length counter
- [x] Noise channel with LFSR
  - [x] Configurable period and envelope
  - [x] Length counter
- [x] DMC (Delta Modulation Channel)
  - [x] Sample playback with configurable rate
  - [x] Delta modulation unit
  - [x] IRQ generation
- [x] Audio mixing with proper channel volumes
- [x] Frame counter with 4-step and 5-step modes

### Mapper Support
- [x] NROM (Mapper 0)
- [x] MMC1 (Mapper 1)
- [ ] MMC3 (Mapper 4)
- [ ] Other common mappers

### Planned Features
- [ ] Save states
- [ ] Controller support (keyboard only for now)
- [ ] Game Genie/Action Replay codes
- [ ] Debugger interface
- [ ] Rewind functionality

### Implementation Details
- Memory access with page boundary detection
- Proper interrupt handling with correct timing
- Support for all 6502 addressing modes
- Comprehensive test coverage for CPU instructions
- Cycle-accurate PPU timing
- Accurate memory mapping for different mappers

## Screenshot

*Screenshot will be added when the emulator has video output*

## Requirements

- Rust (latest stable version recommended)
- GTK4 and development files
- pkg-config

### Ubuntu/Debian

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    libgtk-4-dev \
    libgdk-pixbuf2.0-dev \
    libglib2.0-dev \
    libcairo2-dev \
    pkg-config \
    libpango1.0-dev
```

### Arch Linux

```bash
sudo pacman -S --needed \
    base-devel \
    gtk4 \
    pango \
    cairo \
    gdk-pixbuf2 \
    gcc \
    pkgconf \
    gtk4
```

### Fedora

```bash
sudo dnf install \
    gtk4-devel \
    gdk-pixbuf2-devel \
    glib2-devel \
    cairo-devel \
    pango-devel \
    gcc-c++ \
    pkg-config
```

## Building

```bash
# Clone the repository
git clone https://github.com/yourusername/nes83x-rs.git
cd nes83x-rs

# Build in release mode (recommended)
cargo build --release
```

The binary will be available at `target/release/nes83x-rs`.

## Running

```bash
# Run the emulator
cargo run --release -- path/to/rom.nes

# For debugging
RUST_LOG=debug cargo run -- --debug path/to/rom.nes
```

## Controls

- **Arrow Keys**: D-Pad
- **Z**: A Button
- **X**: B Button
- **Enter**: Start
- **Shift**: Select
- **Escape**: Quit

## Project Structure

- `src/cpu/`: 6502 CPU emulation
- `src/nes/`: NES system emulation
- `src/emulator/`: GTK4 UI and emulator frontend
- `src/main.rs`: Entry point and command-line interface

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [NESDev Wiki](https://wiki.nesdev.org) for detailed NES documentation
- [OneLoneCoder NES emulator](https://github.com/OneLoneCoder/olcNES) for inspiration
- [Rust NES Emulator](https://github.com/koute/pinky) for Rust implementation ideas
