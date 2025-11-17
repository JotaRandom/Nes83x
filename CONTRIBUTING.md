# Contributing to NES83x

Thank you for your interest in contributing to NES83x! This document outlines the guidelines and processes for contributing to this NES emulator project.

## Code Style Guide

NES83x follows strict coding standards. All contributions must adhere to these requirements:

### Mandatory Requirements

1. **GTK4 Interface**: All graphical interfaces must use GTK4
2. **Rust Official Style**: Code must be formatted with `rustfmt`
3. **Clippy Compliance**: All clippy warnings must be resolved
4. **Documentation**: All public functions must have comprehensive Rustdoc comments
5. **Testing**: Unit tests required for all functions, integration tests for modules

### Code Quality Standards

```bash
# Format code
cargo fmt

# Check for warnings (treated as errors)
cargo clippy -- -D warnings

# Run tests
cargo test

# Generate documentation
cargo doc --no-deps
```

### Style Guidelines

- Follow Rust's official style guide: https://doc.rust-lang.org/1.0.0/style/
- Use `rustfmt` for automatic code formatting
- Use meaningful variable and function names
- Prefer functions over complex one-liners
- Avoid deeply nested code; use early returns
- Prefer pattern matching over if-else chains
- Use iterators and functional constructs over loops
- Avoid magic numbers; use constants or enums
- Keep functions focused and single-purpose

## Development Setup

### Prerequisites

#### Linux (Primary Target)
```bash
# Ubuntu/Debian
sudo apt-get install -y build-essential libgtk-4-dev libgdk-pixbuf2.0-dev \
    libglib2.0-dev libcairo2-dev pkg-config libpango1.0-dev

# Arch Linux
sudo pacman -S --needed base-devel gtk4 pango cairo gdk-pixbuf2 gcc pkgconf
```

#### Windows (Secondary Target)
Use WSL2 with Arch Linux for best compatibility.

### Building

```bash
# Clone repository
git clone https://github.com/yourusername/nes83x-rs.git
cd nes83x-rs

# Build in debug mode
cargo build

# Build in release mode
cargo build --release

# Run tests
cargo test

# Check code quality
cargo clippy -- -D warnings
```

## Contribution Process

### 1. Fork and Clone
```bash
# Fork the repository on GitHub
# Clone your fork
git clone https://github.com/yourusername/nes83x-rs.git
cd nes83x-rs

# Add upstream remote
git remote add upstream https://github.com/original/nes83x-rs.git
```

### 2. Create Feature Branch
```bash
# Create and switch to feature branch
git checkout -b feature/your-feature-name

# Or for bug fixes
git checkout -b bugfix/issue-description
```

### 3. Development Workflow

#### Before Coding
```bash
# Ensure you're up to date
git pull upstream master

# Run quality checks on existing code
cargo fmt --check
cargo clippy -- -D warnings
cargo test
```

#### During Development
```bash
# Make small, focused commits
git add -p
git commit -m "feat: add specific feature description"

# Run quality checks frequently
cargo fmt
cargo clippy -- -D warnings
cargo test
```

#### Commit Guidelines
- Use conventional commit format: `type(scope): description`
- Types: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`
- Keep commits small and focused
- Write clear, descriptive commit messages

### 4. Testing

#### Unit Tests
```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_functionality() {
        // Test implementation
        assert_eq!(result, expected);
    }
}
```

#### Integration Tests
```rust
#[cfg(test)]
mod integration_tests {
    use super::*;

    #[test]
    fn test_full_emulation_cycle() {
        // Integration test implementation
    }
}
```

### 5. Documentation

#### Function Documentation
```rust
/// Brief description of what the function does.
///
/// Detailed explanation of parameters, return values,
/// and any side effects.
///
/// # Examples
///
/// ```
/// // Example usage
/// let result = function_name(param);
/// ```
pub fn function_name(param: Type) -> ReturnType {
    // Implementation
}
```

#### Module Documentation
```rust
//! # Module Name
//!
//! Brief description of the module's purpose.
//!
//! Detailed explanation of the module's functionality
//! and how it fits into the overall system.
```

### 6. Pull Request Process

#### Before Submitting
```bash
# Final quality checks
cargo fmt --check
cargo clippy -- -D warnings
cargo test
cargo doc

# Update branch
git pull upstream master
git rebase upstream/master
```

#### Pull Request Requirements
- [ ] Code formatted with `rustfmt`
- [ ] All clippy warnings resolved
- [ ] Tests pass
- [ ] Documentation updated
- [ ] Commit messages follow conventional format
- [ ] PR description clearly explains changes

#### Pull Request Template
```markdown
## Description
Brief description of the changes made.

## Type of Change
- [ ] Bug fix
- [ ] New feature
- [ ] Breaking change
- [ ] Documentation update

## Testing
- [ ] Unit tests added/updated
- [ ] Integration tests added/updated
- [ ] Manual testing performed

## Checklist
- [ ] Code follows style guidelines
- [ ] Documentation updated
- [ ] Tests pass
- [ ] Clippy warnings resolved
```

## Code Review Process

### Review Criteria
- Code style compliance
- Test coverage
- Documentation quality
- Performance considerations
- Security implications

### Review Comments
- Be constructive and specific
- Suggest improvements, don't just criticize
- Reference coding standards when applicable
- Acknowledge good practices

## Issue Reporting

### Bug Reports
- Use the bug report template
- Include reproduction steps
- Provide system information
- Attach relevant logs

### Feature Requests
- Clearly describe the proposed feature
- Explain the use case and benefits
- Consider implementation complexity

## Community Guidelines

- Be respectful and inclusive
- Focus on constructive feedback
- Help newcomers learn
- Maintain professional communication

## License

By contributing to NES83x, you agree that your contributions will be licensed under the MIT License.

## Getting Help

- Check existing issues and documentation first
- Use clear, descriptive titles for new issues
- Provide context and examples
- Be patient when waiting for responses

Thank you for contributing to NES83x! 🎮