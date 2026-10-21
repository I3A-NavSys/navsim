# Changelog

## [2.0.0] - 2026-10-21

### Added
- Standard MAVLink v2 communication protocol over UDP using `NAVSIM_MANUAL_CONTROL` messages (ID 12004, 22 bytes payload) at 10 Hz.
- Dynamic point-to-point UDP transmission directly to UAV RX ports (`15000 + uav_idx`).
- Automatic UAV fleet detection via USD stage inspection (`find_navsim_drone_prims` and naming patterns `UAV_*`, `dron_*`) without requiring manual prim tagging.
- Identification of official `uas_id` from USD attribute `NavSim:id` or prim name across UI and secondary viewports.
- Full operational flight state machine: `DISARMED` (0), `MANUAL` (1), `AUTO` (2), `HOVER` (3), and `SYSTEM_FAILURE` (4) with color-coded live state labels.
- Synchronized bidirectional state reporting with NavSim core (including auto-disarm upon mission completion and collision detection).
- Instant flight mode trigger buttons in UI (`Manual`, `Hover`, `Auto`, `Disarm`) and hardware shortcuts (joystick trigger / Spacebar toggle).
- Isolated secondary Viewport (`NavSim_ChaseCamera_Viewport`) titled `NavSim - <uas_id>`, preserving primary Viewport 1 completely intact.
- Automatic camera distance, height, and focal calibration based on the 3D bounding box of the UAV (aerotaxis, minidrones, and standard drones).
- Clean viewport mode: automatically hides Menubar, HUD statistics (FPS, VRAM, memory, resolution), origin, axes, and grid lines (`fill_frame = True`).
- Camera reset shortcut to restore default chase position (joystick Button 2 or `R` / `Numpad 5`).
- Independent vertical (pitch) and horizontal (yaw) camera axis inversion settings.
- Interactive 2x6 button tester matrix (buttons 1 to 12) with real-time green illumination upon physical button press.
- Comprehensive technical guide and user manual in `docs/README.md` with system architecture diagrams, state machines, control mappings, step-by-step instructions, and embedded screenshots in `docs/img/`.

### Changed
- Extension title updated to "NavSim - Joystick" (v2.0.0).
- Communication architecture completely overhauled from legacy internal Kit event bus to standard MAVLink v2 UDP SITL.
- Base RX port updated from legacy 14550 to standard NavSim 15000 (`15000 + uav_idx`).
- Reorganized extension directory to canonical NVIDIA Omniverse layout (`config/`, `data/`, `docs/`, `manual_controller_python/`).
- Optimized documentation screenshot resolutions for seamless display within Isaac Sim's Extension Overview window.

### Removed
- Removed legacy requirement of `NavSim:Manipulable` USD attribute on prims.
- Removed deprecated internal event-based control logic (`UAV_EVENT`).
- Removed obsolete `README.txt` and legacy root `README.md` to establish `docs/README.md` as the single source of truth.
- Removed obsolete v1.0 preview image (`data/preview.png`), replacing it with a transparent asset to prevent default Omniverse placeholder artifacts.

## [1.1.0] - 2024-10-08
- Controls power section renamed to Control
- Parameters from Control section renamed
- UI distribution reorganized
- Added checkbox to be able to start the control of a UAV with its rotors on
- Added support to dynamic changes in parameters during simulation time
- Removed support to FollowDroneCamera
- Added support to FollowVelocityCamera
- Now we build our own camera
- Added ability to move the camera
- Added invert camera movement option
- Parts of the mapping from both the joystick and the keyboard have been modified to support the camera movement
- Added extra keyboard mapping for those who don't have a keyboard with NUMPAD section (check Overview part)

## [1.0.2] - 2024-10-08
- Added joystick mapping explanation to the documentation

## [1.0.1] - 2024-09-12
- Enhanced ui visuals
- Updated icon, preview and readme data

## [1.0.0] - 2024-09-04
- Complete joystick controller logic
- Hability to track prim's position
- Real-simulation time velocities plots
- Keyboard control support
