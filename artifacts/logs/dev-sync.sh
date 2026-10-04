#!/bin/zsh
# Usage: [OVERLAY=py2] dev-sync.sh — copies the binding the build tree holds into the overlay
# package under artifacts/dev/$OVERLAY (default py), which dev-run.sh puts first on sys.path. The
# installed binding in .venv is not touched, so an arm running on it is not disturbed. Never sync
# an overlay while a dev run on it is in progress; use a second overlay name instead.
set -eu
cd /Users/michaelfeldmeier/Documents/GitHub/scpd-phase-4
overlay=artifacts/dev/${OVERLAY:-py}
mkdir -p $overlay/mqt
if [[ ! -d $overlay/mqt/scpd ]]; then
  cp -R .venv/lib/python3.13/site-packages/mqt/scpd $overlay/mqt/scpd
fi
cp build/cp311-abi3-macosx_15_0_arm64/Release/bindings/pyscpd.abi3.so $overlay/mqt/scpd/pyscpd.abi3.so
codesign -s - --force $overlay/mqt/scpd/pyscpd.abi3.so 2>/dev/null
# The Python side of the package may have moved too; keep the overlay's .py files current.
rsync -a --exclude 'pyscpd.abi3.so' --exclude '__pycache__' python/mqt/scpd/ $overlay/mqt/scpd/
echo "overlay $overlay synced: $(ls -la $overlay/mqt/scpd/pyscpd.abi3.so | awk '{print $6, $7, $8}')"
