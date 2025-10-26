# NES83x - NES Emulator in Rust

A NES emulator written in Rust with GTK4 for the user interface.

## Features

- [x] Basic CPU emulation (6502)
- [ ] PPU emulation (Picture Processing Unit)
- [ ] APU emulation (Audio Processing Unit)
- [ ] Mapper support (currently only NROM)
- [ ] Save states
- [ ] Controller support (keyboard only for now)
- [ ] Game Genie/Action Replay codes

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
