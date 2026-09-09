# 0001 — Bring-your-own-key MILP solver

- **Status:** Accepted
- **Date:** 2026-08-27

## Context

Two pipeline stages, Assignment and Global, are mixed-integer programs. The
prototype used Gurobi with no fallback, so without a commercial licence the tool
did not run at all — including in continuous integration.

MQT SCPD ships on PyPI. A wheel cannot link Gurobi: it is not present on build
machines and redistribution is not permitted.

## Decision

The solver backend is selected **at run time**.

- **HiGHS** is vendored through FetchContent and linked into `MQT::ScpdMilp`. It
  is the default, so `pip install mqt-scpd` is fully functional with no licence.
- **Gurobi** is reached through `gurobipy`, which the user installs themselves
  and which reads `GRB_LICENSE_FILE`. The C++ `Model` emits MPS; the Python
  backend solves and returns values by variable name.
- Selection through `SCPD_SOLVER=auto|highs|gurobi` in the environment or a
  `.env` file, overridable in `config.toml`. `mqt-scpd doctor` reports which
  solvers are visible and why.

MPS covers everything both models need: binary and integer variables, bounds,
linear constraints, and a single linear objective. Write, solve and read back is
milliseconds against models of a few thousand variables.

## Alternatives considered

**A compile-time `-DSCPD_WITH_GUROBI=ON` flag.** Rejected: it serves only people
building from source, which is not bring-your-own-key. Wheel users would get a
package whose planning stages raise.

**`dlopen` of `libgurobi.so` from C++.** Keeps the solve in process, but needs
hand-written symbol loading and Gurobi headers at build time, for stages that
are roughly 10 percent of wall time. Not worth the machinery.

**Drop the mixed-integer programs for heuristics.** Rejected: unacceptable
quality risk against published results.

**Port to an open solver and drop Gurobi entirely.** Rejected: loses Gurobi's
performance on the largest instances, which matters for publication runs.

## Consequences

- Continuous integration and unlicensed users get a working tool.
- The `gurobipy` distribution on PyPI carries a size-limited licence, so the
  larger benchmarks need the user's own licence. That is the intended behaviour,
  not a defect.
- Backend agreement on small benchmarks becomes a test: if HiGHS and gurobipy
  disagree on objective value, the abstraction is wrong.
- If HiGHS cannot close the largest assignment model, the documented fallback is
  HiGHS for continuous integration and small chips, Gurobi for publication runs.
