# Changelog

## [2.0.0] - 2026-10-21

### Added
- Native vectorized dynamic and inertial physics engine using Isaac Sim PhysX and `RigidPrim` APIs for heterogeneous drone fleets.
- WGS-84 Georeferencing subsystem mapping Cartesian Isaac Sim coordinates (ENU) to ellipsoidal coordinates (latitude, longitude, ellipsoidal altitude HAE).
- Multi-source georeference detection supporting Omniverse Geospatial metadata, Cesium Georeference prims, and custom USD attributes, with instant fallback to UCLM Albacete Campus.
- UTC Chronoreferencing engine ($T_0$) synchronizing virtual simulation time ($t_{\text{sim}}$) with absolute UTC time for regulatory ASTM F3411-22a Direct Remote ID timestamps.
- Fleet validation engine complying with international standards ASTM F3411-22a and EUROCAE ED-282 (EASA U-space), validating `uas_id`, `id_type`, `ua_type`, and `operator_id`.
- Automatic detection and prevention of duplicate UAS IDs before simulation execution (`timeline.stop()`) to ensure operational safety.
- Full MAVLink v2 UDP communication stack with dedicated reception ports (`15000 + idx`) and transmission ports (`16000 + idx`).
- 4D trajectory engine supporting degree-5 Taylor polynomials (`crackle`) with sub-millisecond evaluation and linear state-feedback tracking ($u = H_s - K_x x - K_y E$).
- Continuous telemetry broadcaster emitting 5 MAVLink v2 packets per cycle (`LOCATION`, `BASIC_ID`, `OPERATOR_ID`, `SELF_ID`, and `NAVSIM_FLIGHT_PLAN_STATUS`).
- Configurable telemetry frequency from 0.1 Hz to 100 Hz (default 1.0 Hz regulatory Remote ID rate).
- Native Isaac Sim "NavSim - Consola" graphical interface styled after the Isaac Sim Property Window, with collapsible sections for Mapa, Tiempo, Flota, and Comunicaciones.
- Real-time fleet inspector showing detailed ASTM F3411 metadata, dynamic model origin, and active UDP socket mappings per aircraft.
- 5-state operational flight machine (`DISARMED`, `MANUAL`, `HOVER`, `AUTO`, `SYSTEM_FAILURE`) with auto-disarm landing on vertiports and seamless handoff to NavSim - Joystick.

### Changed
- Extension title updated to "NavSim - Consola" (v2.0.0).
- Standardized canonical directory structure (`config/`, `data/`, `docs/`, `docs/img/`, `navsim_python/`).
- Standardized MAVLink port convention to dedicated RX `15000 + idx` and TX `16000 + idx` (superseding legacy 14550 ports).
- Consolidated documentation into `docs/README.md` as the single authoritative source of truth.
- Preview asset replaced with transparent PNG to avoid default placeholder display in Isaac Sim.

### Removed
- Removed legacy untracked root `README.md`.
- Removed hardcoded prim hierarchy assumptions, enabling discovery of UAVs anywhere in the USD Stage based on `NavSim:` attributes.

## [0.1.0] - 2025-07-21

### Added
- Initial version of NavSim Extension.
