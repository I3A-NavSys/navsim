#!/usr/bin/env bash
# Build the Angular frontend and run the Rust backend of the
# FlightPlan editor.
#
# Usage:
#   ./start.sh                    # default port 8000
#   ./start.sh 9000               # custom port
#   ./start.sh --no-build [port]  # skip `npm run build`
#   ./start.sh --rebuild          # force a fresh rebuild
#
# Environment overrides:
#   PORT         TCP port to bind (default 8000); positional arg wins
#   SKIP_BUILD   non-empty → skip `npm run build`
#   FORCE_REBUILD non-empty → force `npm run build` + `cargo build`
set -euo pipefail

EDITOR_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$EDITOR_DIR"

log() { printf '\033[1;35m==>\033[0m %s\n' "$*"; }

# ---------------------------------------------------------------
# Argument parsing (positional-safe even under `set -u`).
# ---------------------------------------------------------------
PORT_FROM_ENV="${PORT:-8000}"
SKIP_BUILD=false
FORCE_REBUILD=false

for arg in "$@"; do
    case "$arg" in
        --no-build)   SKIP_BUILD=true  ;;
        --rebuild)    FORCE_REBUILD=true; SKIP_BUILD=false ;;
        -h|--help)
            sed -n '2,15p' "$0"
            exit 0
            ;;
        --*)          echo "unknown flag: $arg" >&2; exit 2 ;;
        *)
            # First non-flag arg → port
            PORT_FROM_ENV="$arg"
            ;;
    esac
done

PORT="$PORT_FROM_ENV"
if [ -n "${SKIP_BUILD_ENV:-${SKIP_BUILD:-}}" ] \
   && [ "$SKIP_BUILD" = false ] \
   && [ -n "${SKIP_BUILD_ENV:-}" ]; then
    SKIP_BUILD=true
fi
if [ -n "${FORCE_REBUILD_ENV:-}" ]; then
    FORCE_REBUILD=true
fi
unset PORT_FROM_ENV

# ---------------------------------------------------------------
# 1. Build the Angular bundle if it isn't there yet, if any
#    source file is newer than the existing bundle, or if
#    --rebuild was passed. This guarantees `start.sh` always
#    serves the latest frontend code.
# ---------------------------------------------------------------
NEED_FRONTEND_BUILD=false
if [ "$FORCE_REBUILD" = true ] || [ "$SKIP_BUILD" = false ]; then
    if [ "$FORCE_REBUILD" = true ]; then
        NEED_FRONTEND_BUILD=true
    elif [ ! -d frontend/dist ] || [ ! -f frontend/dist/browser/index.html ]; then
        NEED_FRONTEND_BUILD=true
    else
        # Source-edit detection: rebuild whenever any input to the
        # Angular compiler is newer than the existing bundle, so
        # edits to the .ts/.html/.scss sources (or to the build
        # config / npm deps) are always reflected in the next run.
        NEWEST_SRC_FILE=$(find \
            frontend/src \
            frontend/angular.json \
            frontend/package.json \
            frontend/package-lock.json \
            frontend/proxy.conf.json \
            frontend/tsconfig.app.json \
            frontend/tsconfig.json \
            -type f -printf '%T@ %p\n' 2>/dev/null \
            | sort -nr | head -1 | cut -d' ' -f2-)
        NEWEST_BUNDLE_FILE=$(find frontend/dist -type f -printf '%T@ %p\n' 2>/dev/null \
            | sort -nr | head -1 | cut -d' ' -f2-)

        if [ -z "$NEWEST_BUNDLE_FILE" ]; then
            NEED_FRONTEND_BUILD=true
        elif [ -n "$NEWEST_SRC_FILE" ] \
             && [ "$NEWEST_SRC_FILE" -nt "$NEWEST_BUNDLE_FILE" ]; then
            NEED_FRONTEND_BUILD=true
        fi
    fi
fi

if [ "$NEED_FRONTEND_BUILD" = true ]; then
    log "Building Angular frontend"
    pushd frontend >/dev/null
    if [ ! -d node_modules ]; then
        log "Installing npm dependencies (~2 minutes the first time)"
        npm install --no-audit --no-fund
    fi
    npm run build
    popd >/dev/null
else
    log "Angular bundle present, skipping build (use --rebuild to force)"
fi

# ---------------------------------------------------------------
# 2. Build the Rust backend if it isn't built yet (or if
#    --rebuild was passed).
# ---------------------------------------------------------------
NEED_BACKEND_BUILD=false
if [ "$FORCE_REBUILD" = true ]; then
    NEED_BACKEND_BUILD=true
elif [ ! -f backend/target/release/flight-plan-editor ] \
     && [ ! -f backend/target/debug/flight-plan-editor ]; then
    NEED_BACKEND_BUILD=true
fi

if [ "$NEED_BACKEND_BUILD" = true ]; then
    log "Building Rust backend"
    pushd backend >/dev/null
    if [ "$FORCE_REBUILD" = true ] || [ ! -d target ]; then
        cargo build --release
    else
        # Release target missing — fall back to dev build.
        cargo build
    fi
    popd >/dev/null
else
    log "Rust binary present, skipping build (use --rebuild to force)"
fi

# ---------------------------------------------------------------
# 3. Run.
# ---------------------------------------------------------------
log "Starting FlightPlan editor on http://127.0.0.1:$PORT"
log "  Ctrl-C to stop."
log ""

pushd backend >/dev/null
exec cargo-1.91 run --release -- --port "$PORT"
popd >/dev/null
