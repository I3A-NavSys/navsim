#!/usr/bin/env bash
# Build the Rust crate and run the Python-vs-Rust comparison harness.
#
# Usage:
#   ./run.sh                  # build (release) and run with default sizes
#   ./run.sh 100,500,2000     # custom sizes
set -euo pipefail

cd "$(dirname "$0")"

SIZES="${1:-10,50,200,500}"

# Prefer the project venv if it exists (it has numpy + matplotlib).
PY_BIN="python3"
for cand in ../../.venv/bin/python ../../../.venv/bin/python /opt/venv/bin/python; do
    if [ -x "$cand" ]; then
        PY_BIN="$cand"
        break
    fi
done
# Also honour $SIZES and a user-provided $PYTHON_BIN.
if [ -n "${PYTHON_BIN:-}" ]; then
    PY_BIN="$PYTHON_BIN"
fi
echo "=== cargo build --release ==="
cargo build --release

echo
echo "=== $PY_BIN benchmarks/run_comparison.py  (sizes=$SIZES) ==="
SIZES="$SIZES" "$PY_BIN" benchmarks/run_comparison.py

