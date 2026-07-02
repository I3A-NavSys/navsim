"""
UI_editor backend.

A minimal HTTP server built on Python's stdlib `http.server` that:

  * Serves the static Angular bundle (UI_editor/frontend/dist).
  * Exposes a JSON REST API under /api/... that wraps the
    `uspace.flight_plan` package so the Angular editor can drive
    a real FlightPlan object.

The server is single-process and single-threaded on purpose; it is
meant to be run locally for development, not in production.
"""

from __future__ import annotations

import json
import os
import sys
import traceback
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse

# --- Path setup -------------------------------------------------------------
HERE        = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT   = os.path.abspath(os.path.join(HERE, "..", ".."))
# `ng build` with the @angular-devkit/build-angular:application builder
# writes the bundle to `dist/browser`.  Older builders wrote directly to
# `dist`.  Support both layouts.
DIST_DIR_CANDIDATES = [
    os.path.abspath(os.path.join(HERE, "..", "frontend", "dist", "browser")),
    os.path.abspath(os.path.join(HERE, "..", "frontend", "dist")),
]
DIST_DIR = next((d for d in DIST_DIR_CANDIDATES if os.path.isdir(d)), DIST_DIR_CANDIDATES[0])

for p in (REPO_ROOT, os.path.join(REPO_ROOT, "uspace")):
    if p not in sys.path:
        sys.path.insert(0, p)

from uspace.flight_plan.flight_plan_new import FlightPlan    # noqa: E402

# --- In-memory flight plan state -------------------------------------------
# The editor manipulates a single flight plan at a time.  We keep it here so
# every HTTP request can hit the same object.
STATE: dict = {
    "fp": FlightPlan(),
    "next_id": 0,
}


def _new_id(prefix: str = "WP") -> str:
    STATE["next_id"] += 1
    return f"{prefix}{STATE['next_id'] - 1}"


def _serialise_fp(fp: FlightPlan) -> dict:
    """Return a JSON-friendly snapshot of the flight plan."""
    return {
        "length":   int(fp.length),
        "id":       str(fp.id),
        "priority": int(fp.priority),
        "radius":   float(fp.radius),
        "waypoints": [
            {
                "id":     str(wp.id),
                "time":   float(wp.time),
                "pos":   [float(x) for x in wp.pos],
                "vel":   [float(x) for x in wp.vel],
                "acel":  [float(x) for x in wp.acel],
                "jerk":  [float(x) for x in wp.jerk],
                "snap":  [float(x) for x in wp.snap],
                "crakle":[float(x) for x in wp.crakle],
            }
            for wp in fp.waypoints
        ],
    }


def _serialise_trace(trace) -> dict:
    """Return a JSON-friendly snapshot of a trace (numpy array)."""
    return {
        "shape":  [int(s) for s in trace.shape],
        "t":      [float(x) for x in trace[:, 0].tolist()],
        "pos":    [[float(x) for x in row] for row in trace[:, 1:4].tolist()],
        "vel":    [[float(x) for x in row] for row in trace[:, 4:7].tolist()],
        "acel":   [[float(x) for x in row] for row in trace[:, 7:10].tolist()],
        "jerk":   [[float(x) for x in row] for row in trace[:, 10:13].tolist()],
        "snap":   [[float(x) for x in row] for row in trace[:, 13:16].tolist()],
        "crakle": [[float(x) for x in row] for row in trace[:, 16:19].tolist()],
    }


def _empty_trace_dict() -> dict:
    """Return the shape of an empty trace, used when a plan has < 2 WPs."""
    return {
        "shape":  [0, 19],
        "t":      [],
        "pos":    [],
        "vel":    [],
        "acel":   [],
        "jerk":   [],
        "snap":   [],
        "crakle": [],
    }


def _serialise_fp_with_trace(fp: FlightPlan) -> dict:
    """Combined serialisation: flight plan snapshot + computed trace.

    All the editor mutations route through this so the frontend can
    keep the displayed plan and the displayed trace in sync after a
    single round-trip.
    """
    plan = _serialise_fp(fp)
    if fp.length < 2:
        plan["trace"] = _empty_trace_dict()
    else:
        plan["trace"] = _serialise_trace(fp.trace(0.1))
    return plan


# ---------------------------------------------------------------------------
# Route handlers (return dict, status code, [content-type])
# ---------------------------------------------------------------------------

def _api_not_found(_payload, _handler):
    return {"error": "not found"}, HTTPStatus.NOT_FOUND


ROUTES: dict = {}


def route(method: str, path: str):
    """Tiny routing decorator. Stores the callables in `ROUTES`."""
    def wrap(fn):
        ROUTES[(method, path)] = fn
        return fn
    return wrap


@route("GET", "/api/health")
def health(_payload, _h):
    return {"status": "ok", "length": STATE["fp"].length}, HTTPStatus.OK


@route("GET", "/api/flightplan")
def get_fp(_payload, _h):
    return _serialise_fp(STATE["fp"]), HTTPStatus.OK


@route("POST", "/api/flightplan/reset")
def reset_fp(_payload, _h):
    STATE["fp"] = FlightPlan()
    STATE["next_id"] = 0
    return _serialise_fp(STATE["fp"]), HTTPStatus.OK


@route("POST", "/api/waypoint")
def add_waypoint(payload, _h):
    """
    Add a waypoint and return the updated plan + trace.

    Payload keys:
      * id          (str)         : optional, auto-generated if missing
      * time        (float)       : required
      * pos         [x, y, z]     : required
      * vel         [vx, vy, vz]  : optional, default [0, 0, 0]
    """
    if payload is None:
        payload = {}
    time = payload.get("time")
    pos  = payload.get("pos")
    vel  = payload.get("vel", [0.0, 0.0, 0.0])
    wid  = payload.get("id") or _new_id()
    if time is None or pos is None:
        return {"error": "time and pos are required"}, HTTPStatus.BAD_REQUEST
    STATE["fp"].add_waypoint(
        id=wid, time=float(time), pos=list(pos), vel=list(vel)
    )
    STATE["fp"].connect_waypoints()
    return _serialise_fp_with_trace(STATE["fp"]), HTTPStatus.OK


@route("PATCH", "/api/waypoint")
def update_waypoint(payload, _h):
    """
    Update a waypoint's pos, vel and/or time in place.

    Payload:
      * id (str)            : required
      * pos  [x, y, z]      : optional
      * vel  [vx, vy, vz]   : optional
      * time (float)        : optional — if provided, the waypoint is
                              re-inserted at the new time position so the
                              flight plan stays sorted.

    Returns the updated FlightPlan AND the freshly computed trace
    (connected + sampled) so the client can update both in one go.
    """
    if payload is None or "id" not in payload:
        return {"error": "id is required"}, HTTPStatus.BAD_REQUEST
    wid = payload["id"]
    fp  = STATE["fp"]
    idx = fp.get_idx_by_id(wid)
    if idx is None:
        return {"error": f"unknown waypoint {wid}"}, HTTPStatus.NOT_FOUND

    # In-place pos/vel edits are cheap.
    if "pos" in payload:
        fp.waypoints[idx].pos = [float(x) for x in payload["pos"]]
    if "vel" in payload:
        fp.waypoints[idx].vel = [float(x) for x in payload["vel"]]

    # Time changes may require re-sorting. The Python FlightPlan keeps
    # `waypoints` and `time_waypoints` strictly in sync, so we cannot
    # just mutate `wp.time` and leave the list order broken.
    if "time" in payload:
        new_t = float(payload["time"])
        wp = fp.waypoints[idx]
        if wp.time != new_t:
            wp.time = new_t
            # Rebuild the sorted `time_waypoints` array and the
            # `ids_to_idx` map so `bisect_left/right` still work.
            fp.time_waypoints = sorted(w.time for w in fp.waypoints)
            fp.ids_to_idx = {w.id: i for i, w in enumerate(fp.waypoints)}

    # Reconnect (recompute jerk/snap/crakle) and recompute the trace
    # in a single response — the frontend uses this to keep the
    # displayed state coherent after every drag tick.
    fp.connect_waypoints()
    return _serialise_fp_with_trace(fp), HTTPStatus.OK


@route("DELETE", "/api/waypoint")
def delete_waypoint(payload, _h):
    if payload is None or "id" not in payload:
        return {"error": "id is required"}, HTTPStatus.BAD_REQUEST
    fp  = STATE["fp"]
    idx = fp.get_idx_by_id(payload["id"])
    if idx is None:
        return {"error": f"unknown waypoint {payload['id']}"}, HTTPStatus.NOT_FOUND
    fp.remove_waypoint(idx=idx)
    fp.connect_waypoints()
    return _serialise_fp_with_trace(fp), HTTPStatus.OK


@route("POST", "/api/connect")
def connect(payload, _h):
    """Reconnect every adjacent waypoint pair and return plan + trace."""
    STATE["fp"].connect_waypoints()
    return _serialise_fp_with_trace(STATE["fp"]), HTTPStatus.OK


@route("POST", "/api/sync")
def sync(payload, _h):
    """
    Re-fetch the current FlightPlan and its trace in a single response.
    This is the canonical "Refresh" call the editor uses to make sure
    the displayed state matches the server's state.
    """
    if payload is None:
        payload = {}
    if payload.get("connect", True):
        STATE["fp"].connect_waypoints()
    return _serialise_fp_with_trace(STATE["fp"]), HTTPStatus.OK


@route("GET", "/api/trace")
def trace(payload, _h):
    dt = 0.05
    if payload and "dt" in payload:
        dt = float(payload["dt"])
    if STATE["fp"].length < 2:
        return _empty_trace_dict(), HTTPStatus.OK
    return _serialise_trace(STATE["fp"].trace(dt)), HTTPStatus.OK


# ---------------------------------------------------------------------------
# HTTP handler
# ---------------------------------------------------------------------------
class Handler(BaseHTTPRequestHandler):
    # Quieter logs.
    def log_message(self, fmt, *args):  # noqa: N802
        sys.stderr.write("%s - - %s\n" % (self.address_string(), fmt % args))

    # ----- helpers -----
    def _send_json(self, status: int, body) -> None:
        data = json.dumps(body).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(data)

    def _send_file(self, path: str) -> None:
        if not os.path.isfile(path):
            self.send_response(HTTPStatus.NOT_FOUND)
            self.end_headers()
            return
        ctype = (
            "text/html; charset=utf-8" if path.endswith(".html")
            else "application/javascript" if path.endswith(".js")
            else "text/css"               if path.endswith(".css")
            else "application/octet-stream"
        )
        with open(path, "rb") as fh:
            data = fh.read()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _read_json(self):
        length = int(self.headers.get("Content-Length", 0) or 0)
        if length == 0:
            return None
        body = self.rfile.read(length)
        try:
            return json.loads(body)
        except json.JSONDecodeError:
            return None

    # ----- HTTP verbs -----
    def do_OPTIONS(self):  # noqa: N802
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, PATCH, DELETE, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):    self._dispatch("GET")    # noqa: N802
    def do_POST(self):   self._dispatch("POST")   # noqa: N802
    def do_PATCH(self):  self._dispatch("PATCH")  # noqa: N802
    def do_DELETE(self): self._dispatch("DELETE") # noqa: N802

    def _dispatch(self, method: str) -> None:
        url = urlparse(self.path)
        path = url.path

        # ----- API -----
        if path.startswith("/api/"):
            payload = self._read_json() if method != "GET" else None
            handler = ROUTES.get((method, path), _api_not_found)
            try:
                body, status = handler(payload, self)
            except Exception as exc:  # noqa: BLE001
                body = {
                    "error":  str(exc),
                    "trace":  traceback.format_exc(),
                }
                status = HTTPStatus.INTERNAL_SERVER_ERROR
            self._send_json(status, body)
            return

        # ----- static -----
        # 1. Try the exact path in DIST_DIR.
        rel = path.lstrip("/") or "index.html"
        candidate = os.path.normpath(os.path.join(DIST_DIR, rel))
        if not candidate.startswith(DIST_DIR):
            self.send_response(HTTPStatus.FORBIDDEN)
            self.end_headers()
            return
        if os.path.isfile(candidate):
            self._send_file(candidate)
            return

        # 2. SPA fallback: only for extension-less routes (e.g. /foo/bar).
        #    If the request explicitly asks for a file with an extension
        #    (e.g. /main.js, /styles.css) and it's missing, we return
        #    404 — that is the right answer for a stale build, wrong
        #    hash, etc.
        looks_like_asset = "." in rel.rsplit("/", 1)[-1]
        if looks_like_asset:
            self.send_response(HTTPStatus.NOT_FOUND)
            self.end_headers()
            return

        # 3. Serve index.html for SPA routes.
        idx = os.path.join(DIST_DIR, "index.html")
        if os.path.isfile(idx):
            self._send_file(idx)
        else:
            self._send_json(
                HTTPStatus.NOT_FOUND,
                {
                    "error":  "frontend not built",
                    "hint":   f"run `cd {os.path.join(HERE, '..', 'frontend')}"
                              f" && npm run build`",
                    "path":   path,
                },
            )


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(argv=None) -> int:
    import argparse
    parser = argparse.ArgumentParser(description="UI_editor backend server")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=8000, type=int)
    args = parser.parse_args(argv)

    if not os.path.isdir(DIST_DIR):
        print(f"[!] {DIST_DIR} does not exist yet.", file=sys.stderr)
        for cand in DIST_DIR_CANDIDATES:
            print(f"    tried: {cand}", file=sys.stderr)
        print(f"    The API will still work, but the editor UI will not.", file=sys.stderr)
    else:
        print(f"[+] Serving static files from {DIST_DIR}", flush=True)

    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"[+] UI_editor backend listening on http://{args.host}:{args.port}", flush=True)
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
