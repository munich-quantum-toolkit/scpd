# Upgrade Guide

This document describes breaking changes and how to upgrade. For a complete list
of changes including minor and patch releases, please refer to the
[changelog](CHANGELOG.md).

## [Unreleased]

### CMake 3.28

Building from source now requires CMake 3.28 or newer.

### CMake presets on Windows

All CMake presets now use Ninja. On Windows, remove `-windows` from preset names
when configuring, building, and testing:

| Previous preset   | Replacement |
| ----------------- | ----------- |
| `debug-windows`   | `debug`     |
| `release-windows` | `release`   |

Install Ninja and run CMake from a Visual Studio developer shell for the target
architecture. Use a new build directory if an existing directory uses the Visual
Studio generator.

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd
