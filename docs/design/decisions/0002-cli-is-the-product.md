# 0002 — The CLI is the product

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

Most MQT packages present a Python API as their primary interface. For a
physical-design tool the realistic interaction is a command that consumes a chip
description and produces a layout file, not a scripting session against routing
internals.

## Decision

The command-line interface is the supported, documented product. The Python API
is importable but documented as internal and unstable.

Consequences for the binding layer: `bindings/bindings.cpp` exposes only what
the CLI needs — run the pipeline, run a single stage, read metrics. It does not
attempt a Pythonic mirror of the C++ data model.

The CLI uses the standard library's `argparse`. Subcommands cover the command
set and a third-party framework would not earn its place.

## Alternatives considered

**Python API as the product.** The MQT convention, and it would mean full
Pythonic bindings, complete stubs and API documentation. Rejected as a large
surface for an audience that does not yet exist.

**Both as equal first-class interfaces.** Rejected: the most work, for the same
reason.

## Consequences

- Documentation is a CLI reference, not an API reference.
- Binding surface stays small, so stub regeneration stays cheap.
- If users later do want a scripting interface, the stage interfaces are already
  clean enough to expose. This decision is reversible; the opposite would not
  have been.
