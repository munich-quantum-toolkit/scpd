# 0011 — Hold runtime, then parallelize

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

The 69-qubit benchmark takes 456 seconds, about 78 percent of it in the Dubins
A*. The prototype threads that work with eight hand-rolled thread pools, no
locking around shared grids, and correctness maintained by an "interference
guard" that keeps active threads outside each other's read radius.

Peak memory is roughly 7.5 GB: each router context holds up to 1.87 GB and four
run concurrently.

## Decision

Match current runtime as the acceptance bar for the port. Treat parallelism as a
deliberate later phase, with the stage interfaces designed so it stays possible.

Memory is the exception: fix it *during* the port. One shared read-only obstacle
grid across threads, per-thread scratch only, bit-packed masks where the
prototype used a byte per cell. Target about 2 GB.

## Alternatives considered

**Parallelism as a first-class goal of the rewrite.** Retrofitting threading
onto shared mutable state is the expensive path, so this is tempting. Rejected
because the `const` stage interfaces already prevent the state sharing that
makes retrofits hard, and doing both at once forfeits the ability to attribute a
regression.

**Ignore performance entirely.** Rejected: without a runtime bar, a structurally
clean rewrite can quietly become unusable on the largest chips.

## Consequences

- Memory is fixed early because it is what caps thread count, so the later
  threading work depends on it.
- The interference-guard reasoning is preserved in the porting notes even though
  the mechanism may change: dynamic work stealing was tried and reverted,
  because rip-up-and-reroute needs the same sweep that ripped a neighbour to
  re-route it.
