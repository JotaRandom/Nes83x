#!/bin/bash
# Build script for NES83x - Cross-compilation to Linux

echo "Building NES83x for Linux target..."

# Ensure we have the Linux target
rustup target add x86_64-unknown-linux-gnu

# Build in release mode
cargo build --target x86_64-unknown-linux-gnu --release

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo "Binary location: target/x86_64-unknown-linux-gnu/release/nes83x-rs"
    ls -lh target/x86_64-unknown-linux-gnu/release/nes83x-rs
else
    echo "Build failed!"
    exit 1
fi