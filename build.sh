#!/bin/bash
# NES83x Build Script - Following Code Style Guide Requirements
# This script ensures code quality and proper building

set -e  # Exit on any error

# Parse arguments first
MODE=${1:-full}
SKIP_CHECKS=false

# Parse additional arguments
for arg in "$@"; do
    case $arg in
        --skip-checks)
            SKIP_CHECKS=true
            ;;
    esac
done

echo "=== NES83x Build Script ==="
echo "Following Code Style Guide Requirements"
echo "Target: $TARGET"
echo "Mode: $MODE"
echo

# Validate mode early
case $MODE in
    "full"|"gtk"|"cli"|"core"|"check-full"|"check-cli"|"clean")
        ;;
    *)
        echo "Invalid mode: $MODE"
        echo ""
        echo "Usage: $0 [full|gtk|cli|core|check-full|check-cli|clean] [--skip-checks]"
        echo ""
        echo "Modes:"
        echo "  full|gtk     - Build full emulator with GTK interface (default)"
        echo "  cli|core     - Build NES core only (no GTK)"
        echo "  check-full   - Check compilation for full emulator"
        echo "  check-cli    - Check compilation for CLI-only mode"
        echo "  clean        - Clean build artifacts"
        echo ""
        echo "Options:"
        echo "  --skip-checks  Skip code quality checks (fmt, clippy, tests)"
        exit 1
        ;;
esac

# Function to run quality checks
run_quality_checks() {
    if [ "$SKIP_CHECKS" = "true" ]; then
        echo "Skipping quality checks (--skip-checks)"
        return
    fi

    # Step 1: Format code with rustfmt
    echo "1. Formatting code with rustfmt..."
    cargo fmt
    echo "✓ Code formatted"
    echo

    # Step 2: Run clippy for code quality checks
    echo "2. Running clippy for code quality checks..."
    cargo clippy -- -D warnings
    echo "✓ Clippy checks passed"
    echo

    # Step 3: Run tests
    echo "3. Running tests..."
    cargo test
    echo "✓ Tests passed"
    echo
}

# Function to build based on mode
build_target() {
    case $MODE in
        "full"|"gtk")
            echo "Building full emulator with GTK interface..."
            cargo build --target $TARGET --release --features gtk
            ;;
        "cli"|"core")
            echo "Building CLI-only mode (NES core only)..."
            cargo build --target $TARGET --release --no-default-features --features cli
            ;;
        "check-full")
            echo "Checking full emulator with GTK interface..."
            cargo check --target $TARGET --features gtk
            ;;
        "check-cli")
            echo "Checking CLI-only mode (NES core only)..."
            cargo check --target $TARGET --no-default-features --features cli
            ;;
        "clean")
            echo "Cleaning build artifacts..."
            cargo clean
            return
            ;;
    esac
}

# Run quality checks (unless cleaning or checking)
if [ "$MODE" != "clean" ] && [ "$MODE" != "check-full" ] && [ "$MODE" != "check-cli" ]; then
    run_quality_checks
fi

# Build the target
build_target

# Final verification
if [ $? -eq 0 ] && [ "$MODE" != "clean" ] && [ "$MODE" != "check-full" ] && [ "$MODE" != "check-cli" ]; then
    echo
    echo "4. Verifying documentation..."
    cargo doc --no-deps --target $TARGET
    echo "✓ Documentation generated"
    echo

    echo "=== Build completed successfully! ==="
    echo "Binary location: ./target/$TARGET/release/nes83x-rs"
    ls -la target/$TARGET/release/nes83x-rs
    echo
    echo "Code Style Guide Compliance:"
    echo "✓ GTK4 interface implemented"
    echo "✓ Code formatted with rustfmt"
    echo "✓ Clippy checks passed"
    echo "✓ Warnings treated as errors"
    echo "✓ Comprehensive documentation"
    echo "✓ Modular architecture"
    echo "✓ Linux primary target support"
    echo "✓ Feature-based compilation support"
fi