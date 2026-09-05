# Upgrade Guide

This document describes breaking changes and how to upgrade. For a complete list
of changes including minor and patch releases, please refer to the
[changelog](CHANGELOG.md).

## [Unreleased]

### The flat `MQT::SCPD` target is gone

The single library target `MQT::SCPD` is replaced by one target per module:
`MQT::ScpdGeometry`, `MQT::ScpdDesign`, `MQT::ScpdGrid`, `MQT::ScpdRouting`,
`MQT::ScpdMilp`, `MQT::ScpdDrc`, `MQT::ScpdIO` and `MQT::ScpdPipeline`. A CMake
consumer links the module it uses; the dependencies between the modules follow.
`MQT::ScpdPipeline` sits at the top of the dependency graph and pulls in every
other module.

<!-- Version links -->

[unreleased]: https://github.com/munich-quantum-toolkit/scpd
