# 0034 — The plan lives on its own branch

- **Status:** Accepted
- **Date:** 2026-09-28

## Context

The port proceeds one phase per pull request, carried out by LLM agents. The
phase branches each carried a copy of the design documents and changed it
alongside the code, and `main` carried a trimmed copy, so the plan existed in
several diverging versions: `main` had dropped the links to the documents it did
not carry, and the newest decisions were on a phase branch no one had merged. A
feature pull request that also rewrote the plan was hard to review as either.

## Decision

The design documents — `ARCHITECTURE.md`, `docs/design/`, and the decision
records — live on the branch `review_ready` and nowhere else. A feature pull
request carries code, tests, and user documentation; a change of the plan is a
commit to `review_ready`. `main` carries no design documents, and the agents
that carry out a phase are pointed at `review_ready` in their instructions,
since `AGENTS.md` is generated from the organization's templates.

## Consequences

- An agent reads the plan from `review_ready`, for instance through
  `git worktree add ../scpd-plan review_ready`, and proposes plan changes there.
- A decision taken during a phase is recorded on `review_ready` when the phase's
  pull request is opened, so that the review of the code can refer to it.
- `docs/conf.py` on `main` no longer needs to exclude `docs/design/`.
