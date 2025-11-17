---
trigger: always_on
---
# Basic Information
The programming language used in this project is Rust.
The program is a NES emulator with a graphical interface built using GTK4.
Zip files are used for ROM storage and loading.
Aditionally unzipped files with .nes extension are also supported.
# Platforms
Linux is primarily targeted.
Linux can be emulated with WSL on Windows systems, [archlinux-wsl](https://archlinux.org/wsl/) is already installed and configured.
Windows is secondarily targeted.
macOS is thirdly targeted.
FreeBSD is fourthly targeted.
# Target Audience
The target audience for this project are retro gaming enthusiasts and developers interested in NES emulation and game development and game illiterate.
Target platforms are desktop operating systems such as Windows, Linux and macOS, with a focus on user-friendly graphical interfaces and ease of use and accessibility.
# Project Structure
The project is organized into modules based on functionality, such as CPU, PPU, APU, input handling, and graphical interface.
Each module is further divided into submodules for specific components, such as mappers, memory management, and rendering.
# Code Style Guide
Code is for Rust language
Inteface with other languages is out of scope for now
Graphical interface is thorough GTK4
Follow Rust's official style guide: https://doc.rust-lang.org/1.0.0/style/
Use rustfmt to format code automatically
Use clippy to catch common mistakes and improve code quality
All code shall be well documented and commented
All Code shall be easily underestandable
When possibleuse easy to identify names for variables and everyhting else
Avoid complex one-liners
Prefer using functions to encapsulate logic
When possible, avoid deeply nested code
Prefer early returns to reduce nesting
Prefer using pattern matching over if-else chains
Prefer using iterators and functional programming constructs over loops
Avoid using magic numbers, use constants or enums instead
# Compiler Warnings
Enable all compiler warnings
Treat all warnings as errors
Regularly run clippy to catch common mistakes and improve code quality
# Testing
Write unit tests for all functions and methods
Write integration tests for all modules
Use continuous integration to run tests automatically on every commit
Ensure code coverage is above 90%
# Documentation
Document all public functions and methods using Rustdoc comments
Provide examples for complex functions and methods
Maintain an up-to-date README file with installation and usage instructions
Write a CONTRIBUTING.md file with guidelines for contributing to the project
Wait for compiler to finish before running tests or other commands
# Version Control
Use Git for version control
Write clear and concise commit messages
Commit often with small, focused changes
Follow Gitflow workflow for branching and merging
# Code Reviews
