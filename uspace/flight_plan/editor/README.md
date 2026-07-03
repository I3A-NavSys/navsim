# FlightPlan Editor

Graphical web editor for the `FlightPlan` / `Waypoint` model used in
`navsim/uspace/flight_plan/`. Lets you create, view and modify one or
many flight plans in real time, both in a 3D viewer and as a table,
with live plots of the kinematic derivatives over time.

```
editor/
├── backend/                     Rust + axum backend
│   ├── Cargo.toml
│   ├── src/
│   │   ├── main.rs              # axum router + static fallback
│   │   ├── state.rs             # in-memory plan registry + DTOs
│   │   └── handlers.rs          # every REST endpoint
│   └── .gitignore
├── frontend/                    Angular + Three.js + Chart.js
│   ├── angular.json
│   ├── package.json
│   ├── tsconfig.json
│   ├── tsconfig.app.json
│   ├── proxy.conf.json
│   └── src/
│       ├── index.html
│       ├── main.ts
│       ├── styles.css           # single-source-of-truth colour palette
│       └── app/
│           ├── app.component.ts
│           ├── services/
│           │   ├── flight-plan.service.ts
│           │   ├── mutation-stream.service.ts
│           │   └── simulation-clock.service.ts
│           └── components/
│               ├── viewer3d.component.ts        # Three.js 3D viewer
│               ├── info-panel.component.ts     # plan attributes + WP table
│               ├── derivatives.component.ts    # Chart.js per-axis plots
│               └── timeline.component.ts       # simulation timeline strip
├── start.sh                     # build + run
└── README.md                    # this file
```

## What it does

Three resizable panels plus a timeline strip:

| # | Panel        | Tech        | What you can do                                                          |
|---|--------------|-------------|--------------------------------------------------------------------------|
| 1 | **3D viewer** | Three.js    | Show multiple `FlightPlan`s; click empty space to add a WP; drag a WP sphere to move it; drag a velocity arrow tip to edit the velocity vector; switch axis view with the floating toolbar (`+X / −X / +Y / −Y / +Z / −Z / Iso`) |
| 2 | **Info panel** | HTML/Angular | Plan-level attributes (`priority`, `radius`, `max var lin/ang vel`) + per-waypoint editable table (id, time, pos x/y/z, vel x/y/z); `Connect` re-derives jerk/snap/crackle; `Show all / Hide all` toggles waypoint label visibility |
| 3 | **Derivatives** | Chart.js   | One chart per axis (X / Y / Z); choose which derivatives to overlay (`Velocity`, `Acceleration`, `Jerk`, `Snap`, `Crackle`); waypoint times are marked with labelled dashed lines |
| 4 | **Timeline** (bottom strip) | Angular | Shows every WP of every plan as a coloured tick; drag the playhead to scrub; `▶ / ⏸`, `⟲ Reset`, `speed ×` |

The two dividers (between viewer ↔ info and between info ↔ derivatives)
are draggable to resize the layout.

The status indicator in the top-right reads
`✔ in sync` / `… syncing` / `⚠ <error>` reflecting the backend round
trip.

## Layout

```
┌──────────────────────────────────────────────────────────┐
│  top bar:  brand · plan tabs · status                    │
├──────────────────────────────────┬───────────────────────┤
│  ┌────────────────────────────┐  │  Info panel           │
│  │  3D viewer                 │  │  (plan attrs + WP tbl)│
│  │  (axis toolbar + tools)    │  ├───────────────────────┤
│  └────────────────────────────┘  │  Derivatives          │
│                                  │  (per-axis chart × 3) │
├──────────────────────────────────┴───────────────────────┤
│  Timeline strip ▶ ⏸ ⟲  speed ×   ──●──ticks──            │
└──────────────────────────────────────────────────────────┘
```

## Running it

The first time, install Angular dev tooling and a C toolchain (the Rust
crate compiles three build scripts that need a C linker):

```bash
# 1. One-time system dependencies
sudo apt install build-essential           # for cargo build scripts
# (or `apt install gcc` on smaller boxes)

# 2. Frontend deps
cd editor/frontend
npm install

# 3. Run — builds the Rust backend and starts the server in one step.
cd ..
./start.sh                    # default port 8000
./start.sh 9000               # custom port
SKIP_BUILD=1 ./start.sh       # skip the npm/cargo build when iterating
```

Then open <http://127.0.0.1:8000> in a browser.

`start.sh` will detect the first build and run `npm install` +
`npm run build` + `cargo build --release`, so the cold-start can take a
few minutes. Subsequent runs are essentially instant.

## REST API

All write endpoints return the updated `FlightPlan` together with the
recomputed `Trace` so the client can refresh both views in a single
round-trip — the frontend pushes every change through a
`mutations$ → debounce + distinctUntilChanged + switchMap` pipeline so
out-of-order responses never corrupt the UI (the same anti-race pattern
the older `UI_editor` uses).

| Verb   | Path                                          | Body                                 | Returns                  |
|--------|-----------------------------------------------|--------------------------------------|--------------------------|
| GET    | `/api/plans`                                  | —                                    | `{plans: [...], sim_time}` |
| POST   | `/api/plans`                                  | `{id?, priority, radius, waypoints?}` | `FlightPlan + Trace`     |
| GET    | `/api/plans/:id`                              | —                                    | `FlightPlan + Trace`     |
| PATCH  | `/api/plans/:id`                              | `{priority?, radius?, max_var_*?}`   | `FlightPlan + Trace`     |
| DELETE | `/api/plans/:id`                              | —                                    | `204`                    |
| POST   | `/api/plans/:id/visibility`                   | `{visible}`                          | `FlightPlan + Trace`     |
| POST   | `/api/plans/:id/waypoints`                    | `{id?, time, pos, vel?, fly_over?}`  | `FlightPlan + Trace`     |
| PATCH  | `/api/plans/:id/waypoints/patch`              | `{id, time?, pos?, vel?, …}`         | `FlightPlan + Trace`     |
| DELETE | `/api/plans/:id/waypoints/patch`              | `{id}`                               | `FlightPlan + Trace`     |
| POST   | `/api/plans/:id/connect`                      | —                                    | `FlightPlan + Trace`     |
| GET    | `/api/plans/:id/trace?dt=0.05`                | —                                    | `Trace`                  |
| GET    | `/api/sim`                                    | —                                    | `{sim_time}`             |
| POST   | `/api/sim`                                    | `{reset?: bool, time?: number}`      | `{sim_time}`             |
| GET    | `/api/health`                                 | —                                    | `{status, length}`       |

## Colour palette

The single source of truth is `frontend/src/styles.css` under
`:root { --palette … }`. Re-theme the whole UI from there. The
runtime palette used by the 3D viewer and the chart panel is
`FlightPlanService.PALETTE` and `Viewer3dComponent`'s `palette`
field — both kept in lockstep with the CSS variables.

## Differences from the existing `UI_editor`

|                       | `UI_editor/`                                | `editor/`                                       |
|-----------------------|---------------------------------------------|--------------------------------------------------|
| Backend               | Python stdlib                               | **Rust (axum + tokio)** wrapping the local `flight_plan` crate |
| Viewer                | 2D canvas                                   | **Three.js 3D viewer**                           |
| Multiple plans        | no (one at a time)                          | **yes** (tabs + colour-coded polyline)           |
| Axis views            | only Fit / zoom / pan                       | **`+X / −X / +Y / −Y / +Z / −Z / Iso`** toolbar  |
| Derivatives panel     | two stacked charts (all axes overlaid)      | **one chart per axis** with multi-derivative overlays |
| Simulation            | none                                        | **`▶ / ⏸ / ⟲ Reset / scrubbable timeline`**     |
| Layout                | fixed grid (canvas + table)                 | **resizable** via two draggable dividers         |

## Building / lint

```bash
# Frontend typecheck / lint
cd editor/frontend
npx tsc --noEmit                  # typecheck, no emit
npm run build                      # production build

# Backend check / clippy
cd editor/backend
cargo check
cargo clippy -- -D warnings
cargo build --release
```

## Notes

* The Rust `flight_plan` crate previously had no HTTP layer or
  `serde` derives; the editor adds both. The benchmark binary under
  `navsim/uspace/flight_plan/rust/src/bin/` is untouched and
  continues to work.
* `connect_waypoints()` (the "Connect" button) re-derives the third
  / fourth / fifth kinematic derivatives so the trace stays
  C⁵-continuous between any two adjacent waypoints.
* The simulation is purely client-side interpolation of the trace
  data already returned by the last mutation — the server-side clock
  is only used to share an initial offset between browser tabs.
