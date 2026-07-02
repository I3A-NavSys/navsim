#!/usr/bin/env bash
# Build the Angular bundle and start the UI_editor backend.
#
# Usage:
#   ./start.sh           # default port 8000
#   ./start.sh 9000      # custom port
set -euo pipefail

cd "$(dirname "$0")"

PORT="${1:-8000}"

# --- 1. Build Angular if dist/ is missing or stale -----------------------
if [ ! -d frontend/dist ] || [ ! -f frontend/dist/browser/index.html ]; then
    echo "=== Building Angular frontend (one time) ==="
    pushd frontend >/dev/null
    if [ ! -d node_modules ]; then
        npm install --no-audit --no-fund
    fi
    npm run build
    popd >/dev/null
else
    echo "=== Angular bundle present, skipping build ==="
fi

# --- 2. Start the Python backend -----------------------------------------
echo
echo "=== Starting UI_editor backend on port $PORT ==="
echo "    Open http://127.0.0.1:$PORT in a browser"
echo "    Press Ctrl-C to stop."
echo

PY="${PYTHON_BIN:-python3}"
for cand in ../.venv/bin/python ../../.venv/bin/python /opt/venv/bin/python; do
    if [ -x "$cand" ]; then PY="$cand"; break; fi
done

exec "$PY" backend/server.py --port "$PORT"
