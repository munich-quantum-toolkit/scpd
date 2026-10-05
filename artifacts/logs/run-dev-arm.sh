#!/bin/zsh
# Usage: [CHIPS="…"] [MAX_RELAXATION=8] run-dev-arm.sh <arm> <overlay> [ENV=VAL ...]
# The eight chips one at a time on the overlay binding artifacts/dev/<overlay> (dev-sync.sh),
# each in a run directory of its own under artifacts/dev/<arm>/<chip>, so two arms on two
# overlays can run side by side without touching each other or the installed build.
# Logs into artifacts/logs/<arm>/<chip>.log like run-arm.sh. repair_trials stays what the
# benchmark config says (0); MAX_RELAXATION edits the run directory's copy as run-arm.sh does.
set -u
cd /Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4
arm=$1; overlay=$2; shift 2
mkdir -p artifacts/logs/$arm
for chip in ${=CHIPS:-4q 9q 17q 21q 33q 45q 57q 69q}; do
  run=artifacts/dev/$arm/$chip
  mkdir -p $run
  for f in 00-chip.json 01-capacity.fb 02-global.fb 03-assign.fb 04-corridor.fb 05-detail.fb; do
    cp artifacts/$chip/$f $run/$f
  done
  cp benchmarks/$chip/config.toml $run/config.toml
  if [[ -n "${MAX_RELAXATION:-}" ]]; then
    sed -i '' -E "s/^max_relaxation( *)= .*/max_relaxation\1= ${MAX_RELAXATION}/" $run/config.toml
  fi
  echo "### $chip start $(date '+%H:%M:%S') repair_trials $(grep -E '^repair_trials' $run/config.toml | tr -s ' ' | cut -d' ' -f3) max_relaxation $(grep -E '^max_relaxation' $run/config.toml | tr -s ' ' | cut -d' ' -f3) env: $* (dev overlay $overlay)" > artifacts/logs/$arm/$chip.log
  SCPD_DEV_OVERLAY=artifacts/dev/$overlay /usr/bin/time -p env "$@" .venv/bin/python artifacts/logs/dev-mqt-scpd.py plan -c benchmarks/$chip/config.toml -o $run --stage final -v 1 >> artifacts/logs/$arm/$chip.log 2>&1
  echo "### $chip end $(date '+%H:%M:%S')" >> artifacts/logs/$arm/$chip.log
done
echo "### arm $arm done $(date '+%H:%M:%S')" > artifacts/logs/$arm/DONE
