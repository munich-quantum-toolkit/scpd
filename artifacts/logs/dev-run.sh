#!/bin/zsh
# Usage: [OVERLAY=py2] [REPAIR_TRIALS=N] dev-run.sh <arm> <chip> [ENV=VAL ...] — one chip on
# the overlay binding (dev-sync.sh), in a run directory of its own under artifacts/dev/<chip>, so
# an arm on the installed build is not disturbed. Logs into artifacts/logs/<arm>/<chip>.log like
# run-arm.sh. REPAIR_TRIALS=N as in run-arm.sh; OVERLAY names the overlay (default py).
set -u
cd /Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4
arm=$1; chip=$2; shift 2
mkdir -p artifacts/logs/$arm artifacts/dev/$chip
for f in 00-chip.json 01-capacity.fb 02-global.fb 03-assign.fb 04-corridor.fb 05-detail.fb; do
  cp artifacts/$chip/$f artifacts/dev/$chip/$f
done
cp benchmarks/$chip/config.toml artifacts/dev/$chip/config.toml
if [[ -n "${REPAIR_TRIALS:-}" ]]; then
  sed -i '' -E "s/^repair_trials = .*/repair_trials = ${REPAIR_TRIALS}/" artifacts/dev/$chip/config.toml
fi
spot=$(ps -A -o %cpu,comm -r | awk 'NR>1 && ($2 ~ /mds_stores|mediaanalysisd/) {s+=$1} END {print s"%"}')
echo "### $chip start $(date '+%H:%M:%S') spotlight $spot repair_trials $(grep -E '^repair_trials' artifacts/dev/$chip/config.toml | tr -s ' ' | cut -d' ' -f3) env: $* (dev overlay ${OVERLAY:-py})" > artifacts/logs/$arm/$chip.log
SCPD_DEV_OVERLAY=artifacts/dev/${OVERLAY:-py} /usr/bin/time -p env "$@" .venv/bin/python artifacts/logs/dev-mqt-scpd.py plan -c benchmarks/$chip/config.toml -o artifacts/dev/$chip --stage final -v 1 >> artifacts/logs/$arm/$chip.log 2>&1
echo "### $chip end $(date '+%H:%M:%S')" >> artifacts/logs/$arm/$chip.log
