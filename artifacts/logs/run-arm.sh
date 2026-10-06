#!/bin/zsh
# Usage: run-arm.sh <arm> [ENV=VAL ...]  — runs all eight chips sequentially, one at a time.
#   REPAIR_TRIALS=N ./run-arm.sh <arm> ...  — sets `repair_trials = N` in the run directory's
#   copy of the config (what `plan --stage final` reads); the benchmark configs stay untouched.
#   CHIPS="45q 57q 69q" ./run-arm.sh <arm> ...  — only these chips, in this order.
#   STOP_AFTER=outer ./run-arm.sh <arm> ...  — sets `stop_after` in the run directory's copy.
#   MAX_RELAXATION=8 ./run-arm.sh <arm> ...  — sets `max_relaxation` in the run directory's copy.
#   REFINE_ROUNDS=2 ./run-arm.sh <arm> ...  — sets `feedline_refinement_rounds` in the run
#   directory's copy, adding the key when the benchmark config does not carry it. The fifth
#   phase only runs with STOP_AFTER=refined as well.
set -u
cd /Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4
arm=$1; shift
mkdir -p artifacts/logs/$arm
for chip in ${=CHIPS:-4q 9q 17q 21q 33q 45q 57q 69q}; do
  cp benchmarks/$chip/config.toml artifacts/$chip/config.toml
  if [[ -n "${REPAIR_TRIALS:-}" ]]; then
    sed -i '' -E "s/^repair_trials = .*/repair_trials = ${REPAIR_TRIALS}/" artifacts/$chip/config.toml
  fi
  if [[ -n "${STOP_AFTER:-}" ]]; then
    sed -i '' -E "s/^stop_after( *)= .*/stop_after\1= \"${STOP_AFTER}\"/" artifacts/$chip/config.toml
  fi
  if [[ -n "${MAX_RELAXATION:-}" ]]; then
    sed -i '' -E "s/^max_relaxation( *)= .*/max_relaxation\1= ${MAX_RELAXATION}/" artifacts/$chip/config.toml
  fi
  if [[ -n "${REFINE_ROUNDS:-}" ]]; then
    awk -v n="${REFINE_ROUNDS}" '
      /^feedline_refinement_rounds/ { print "feedline_refinement_rounds = " n; seen = 1; next }
      { print }
      /^refinement_rounds/ && !seen { print "feedline_refinement_rounds = " n; seen = 1 }
    ' artifacts/$chip/config.toml > artifacts/$chip/config.toml.new
    mv artifacts/$chip/config.toml.new artifacts/$chip/config.toml
  fi
  spot=$(ps -A -o %cpu,comm -r | awk 'NR>1 && ($2 ~ /mds_stores|mediaanalysisd/) {s+=$1} END {print s"%"}')
  echo "### $chip start $(date '+%H:%M:%S') spotlight $spot repair_trials $(grep -E '^repair_trials' artifacts/$chip/config.toml | tr -s ' ' | cut -d' ' -f3) env: $*" > artifacts/logs/$arm/$chip.log
  /usr/bin/time -p env "$@" .venv/bin/mqt-scpd plan -c benchmarks/$chip/config.toml -o artifacts/$chip --stage final -v 1 >> artifacts/logs/$arm/$chip.log 2>&1
  echo "### $chip end $(date '+%H:%M:%S')" >> artifacts/logs/$arm/$chip.log
done
echo "### arm $arm done $(date '+%H:%M:%S')" > artifacts/logs/$arm/DONE
