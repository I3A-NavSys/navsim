# UI_editor

A graphical editor for the `FlightPlan` class of `uspace.flight_plan`.

```
UI_editor/
├── backend/
│   └── server.py            # Python stdlib HTTP server (API + static)
├── frontend/
│   ├── package.json
│   ├── angular.json
│   ├── tsconfig.json
│   ├── src/
│   │   ├── index.html
│   │   ├── main.ts
│   │   ├── styles.css
│   │   └── app/
│   │       ├── app.component.ts
│   │       ├── services/
│   │       │   └── flight-plan.service.ts
│   │       └── components/
│   │           ├── canvas.component.ts          # interactive editor canvas
│   │           ├── info-panel.component.ts      # manual waypoint cells + plan info
│   │           └── derivatives.component.ts     # vel/acc/jerk/snap/crakle plot
│   └── dist/                                   # ng build output
├── start.sh                                      # build + run in one step
└── README.md
```

## Functional requirements coverage

| # | Requirement                                                                | Where it lives                                          |
|---|----------------------------------------------------------------------------|---------------------------------------------------------|
| 1 | Generate waypoints **manually** (text cells) **and** by clicking the canvas | `info-panel.component.ts` (table) + `canvas.component.ts` (click → add) |
| 2 | Show / hide the velocity vector of each waypoint                          | `canvas.component.ts` (toolbar checkbox)                |
| 3 | Rotate / scale the velocity vector with the mouse                          | `canvas.component.ts` (`onMouseDown/Move/Up` in `'vel'` tool) |
| 4 | Move the waypoint position with the mouse                                  | `canvas.component.ts` (`onMouseDown/Move/Up` in `'move'` tool) |
| 5 | Show the whole flight plan info + its trace at the same time               | Two-column layout in `app.component.ts` (canvas + info) |
| 6 | Show / hide a derivatives panel with selectable vel/accel/jerk/snap/crakle | `derivatives.component.ts` (checkboxes per derivative) |

## Extra features added after the first round of testing

| Feature                                              | Where it lives                                          |
|------------------------------------------------------|---------------------------------------------------------|
| **Real-time trace update** during drag (every ~20 ms)| `app.component.ts` (`mutations$` + `switchMap`)        |
| **Zoom in / out** on the canvas (mouse wheel)       | `canvas.component.ts` (`onWheel`)                      |
| **Pan** the canvas (shift+drag or middle-button drag)| `canvas.component.ts` (`onMouseDown` → `kind: 'pan'`)  |
| **Zoom buttons** (+ / Fit / −) in the editor toolbar | `app.component.ts` (`.canvas-zoom-bar`)               |
| **Waypoint time markers** in the derivatives chart   | `derivatives.component.ts` (vertical dashed lines + chip legend) |
| **Auto-connect live trace** (default on, toggleable) | `app.component.ts` (`autoConnect` checkbox)           |
| **Status indicator** ("… syncing" / "✔ in sync" / "⚠ <error>") | `app.component.ts` (header right)         |

## Race-condition fix (the most subtle bug)

The original frontend used a `subscribe().subscribe()` pattern: every
UI mutation triggered a chain of three sequential HTTP calls (PATCH
waypoint → POST /connect → GET /api/trace). When the user dragged
rapidly, dozens of these chains were in flight at once and their
responses arrived in arbitrary order, leaving the displayed state
incoherent — sometimes stale, sometimes a mix of different
operations.

The fix has two parts:

1. **A single combined endpoint per mutation.** Every write API
   (`POST /api/waypoint`, `PATCH /api/waypoint`, `DELETE /api/waypoint`,
   `POST /api/connect`, `POST /api/sync`) now returns the updated
   `FlightPlan` **together with the freshly computed trace** in the
   same JSON payload. The frontend only has to apply one response per
   mutation.

2. **A single `switchMap` pipeline in `app.component.ts`.** All UI
   events are pushed onto a `mutations$: Subject<Mutation>`, which
   feeds an RxJS pipeline:

   ```
   mutations$
     .pipe(debounceTime(20), distinctUntilChanged(),
           switchMap(m => dispatch(m)))
   ```

   `switchMap` automatically cancels any in-flight request when a
   newer one arrives, so only the latest response is applied to the
   UI. `debounceTime(20)` coalesces rapid drag-ticks (~50 Hz) into
   one update per ~20 ms, giving a smooth real-time trace.

## Running

The backend is pure-Python (stdlib only) and serves both the API and
the built Angular bundle.

```bash
# 1. Install Angular dev tooling (one time)
cd UI_editor/frontend
npm install

# 2. Build the Angular app in production mode
npm run build

# 3. Run the backend
cd ../backend
python3 server.py --port 8000
```

Or use the convenience script that does all three:

```bash
./start.sh                  # port 8000
./start.sh 9000             # custom port
```

Then open <http://127.0.0.1:8000> in a browser.

## REST API

The backend exposes a JSON API under `/api/...`. **Every write
endpoint returns `{...flightplan, trace}` so the client can keep both
views in sync after a single round-trip.**

| Verb   | Path                          | Body                                                              | Returns                          |
|--------|-------------------------------|-------------------------------------------------------------------|----------------------------------|
| GET    | `/api/flightplan`             | —                                                                 | `FlightPlan` snapshot            |
| POST   | `/api/flightplan/reset`       | —                                                                 | empty plan + empty trace         |
| POST   | `/api/waypoint`               | `{id?, time, pos, vel?}`                                          | plan + trace                     |
| PATCH  | `/api/waypoint`               | `{id, pos?, vel?, time?}`                                        | plan + trace                     |
| DELETE | `/api/waypoint`               | `{id}`                                                            | plan + trace                     |
| POST   | `/api/connect`                | —                                                                 | plan + trace (recomputes jerk/snap/crakle) |
| POST   | `/api/sync`                   | `{connect?}`                                                      | plan + trace (no edits)          |
| GET    | `/api/trace?dt=0.05`          | —                                                                 | standalone `Trace`               |
| GET    | `/api/health`                 | —                                                                 | `{status, length}`               |

The `time` field of `PATCH /api/waypoint` triggers a re-sort of the
waypoint list if the new time would put the WP at a different
position, so the flight plan stays strictly time-sorted after every
edit.

## Controls cheat sheet

| Action                       | Input                              |
|------------------------------|------------------------------------|
| Add a waypoint               | `Add WP` tool + click on canvas    |
| Move a waypoint              | `Move WP` tool + drag              |
| Edit velocity vector         | `Edit Velocity` tool + drag tip    |
| Pan the canvas               | Shift+drag (or middle-button drag) |
| Zoom in / out                | Mouse wheel                        |
| Reset view                   | `Fit` button                       |
| Edit pos / vel / time        | Click cell in the right table      |
| Delete a waypoint            | `×` button in the table row        |
| Toggle trace / velocity      | Toolbar checkboxes                 |
| Force re-connect             | `Connect` button                   |
| Re-fetch server state        | `Refresh trace` button             |
