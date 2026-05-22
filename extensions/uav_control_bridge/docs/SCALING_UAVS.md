# Scaling the UAV Control Bridge to N UAVs

## Overview

The UAV Control Bridge has been refactored to support an arbitrary number of UAVs instead of being hardcoded to exactly two. This document explains what you need to configure in Isaac Sim to make it work with 3, 4, 5, or more UAVs.

## Code Changes Made

### 1. **extension.py** – Dynamic UAV Detection

The bridge now:
- Automatically detects all UAV prims present in the `/World/UAVs/UAV_*` path
- Builds a dynamic mapping of these prims to their physics buffer indices
- Generates the same number of flight plans as UAVs exist in the scene
- Loads them via the selected Viz 08 case (original or resolved)

**Key methods:**
- `_get_ordered_uav_entries()` – Flattens the nested `{operator_id: {uav_id: physics_index}}` structure into a sorted list for deterministic pairing
- `_sort_viz08_plan_id()` – Sorts flight plan IDs numerically (UAV_1, UAV_2, ..., UAV_N) before assignment

**No hardcoding.** The number of UAVs is now determined entirely by what exists in Isaac Sim.

### 2. **viz_08_scenario_data.py** – Scalable Scenario Generation

The generator now:
- Preserves the exact original five routes from the visualizer (UAV_1 through UAV_5)
- Reuses the same reversed routes as the visualizer, skipping the vertical-only case so the scene contains 9 UAVs
- Maintains conflict geometry and timing so the 9-UAV scenario still converges at t=20s around position [200, 200, 100]

**Key function:**
- `_build_generated_route(index, total_uavs)` – Distributes extra UAVs around the conflict point with alternating vertical offsets to avoid coplanar collisions

## Isaac Sim Scene Configuration

### What You Need to Do

For **9 UAVs**, you must:

1. **Create N UAV Prims** under `/World/UAVs/`
   - Each prim must follow the naming convention: `UAV_0`, `UAV_1`, ..., `UAV_8`
   - Example paths:
     ```
     /World/UAVs/UAV_0
     /World/UAVs/UAV_1
     /World/UAVs/UAV_2
     ...
     /World/UAVs/UAV_N
     ```

2. **Set Required Attributes** on each UAV prim
   - `NavSim:operator_id` (string) – e.g., `"UAV_OPERATOR_0"`
   - `NavSim:id` (string) – e.g., `"UAV_0"`, `"UAV_1"`, etc.
   
   These attributes are read by the bridge's `relate_uav_ids_to_physics_buffer()` method to create the physics index mapping.

3. **Use RigidPrim Wildcards**
   - The bridge queries `/World/UAVs/UAV_*` using Isaac Sim's RigidPrimView
   - All matching prims are automatically detected and initialized
   - Order is preserved: physics indices are assigned sequentially

### Example: From 2 UAVs to 5 UAVs

**Before (2 UAVs):**
```
/World/UAVs/UAV_0  (NavSim:operator_id = "UAV_OPERATOR_0", NavSim:id = "UAV_0")
/World/UAVs/UAV_1  (NavSim:operator_id = "UAV_OPERATOR_0", NavSim:id = "UAV_1")
```

**After (5 UAVs):**
```
/World/UAVs/UAV_0  (NavSim:operator_id = "UAV_OPERATOR_0", NavSim:id = "UAV_0")
/World/UAVs/UAV_1  (NavSim:operator_id = "UAV_OPERATOR_0", NavSim:id = "UAV_1")
/World/UAVs/UAV_2  (NavSim:operator_id = "UAV_OPERATOR_0", NavSim:id = "UAV_2")
/World/UAVs/UAV_3  (NavSim:operator_id = "UAV_OPERATOR_0", NavSim:id = "UAV_3")
/World/UAVs/UAV_4  (NavSim:operator_id = "UAV_OPERATOR_0", NavSim:id = "UAV_4")
```

Then in the Python REPL or via the extension, set the environment variable before playing:
```bash
export UAV_CONTROL_BRIDGE_VIZ08_CASE=resolved  # or "original"
```

The bridge will automatically:
1. Detect 9 UAVs in the scene
2. Generate 9 flight plans via `get_viz08_flightplans(case=..., n_uavs=9)`
3. Pair them in order and run the control loop

## Flight Plan Selection

### Environment Variable

Set before starting the simulation:
```bash
export UAV_CONTROL_BRIDGE_VIZ08_CASE=original    # Original (unresolved) scenario
export UAV_CONTROL_BRIDGE_VIZ08_CASE=resolved    # Resolved scenario (default)
```

If your USD places the UAVs under a different prim hierarchy, override the
path expression used by the bridge:
```bash
export UAV_CONTROL_BRIDGE_UAV_PRIM_EXPR="/World/YourScene/Path/UAV_*"
```

If not set, defaults to `"resolved"`.

### Scenario Behavior

- **original**: 9 UAVs with exact routes from Viz 08, including the reversed routes except the vertical-only one.
- **resolved**: Same 9 UAVs, but with kinematic bounding adjustments applied by the conflict resolver.

Both run the same central manager cascade, so extra UAVs beyond five will be resolved if conflicts are detected.

## Performance Notes

- **Caching**: Flight plans are built once at simulation startup (`on_timeline_play`), not on every physics step.
- **Scalability**: The control loop's `update()` method in `UAVControl` processes all physics indices in parallel via NumPy operations, so scaling to 10+ UAVs should not incur linear slowdown.
- **Memory**: Each UAV adds one FlightPlan and control state arrays in `UAVControl`. For reasonable fleet sizes (≤20), this is negligible.

## Debugging

If the bridge fails to detect your UAVs:

1. **Check the prim structure:**
   ```python
   # In Isaac Sim Python console
   from isaacsim.core.prims import RigidPrim
   rp = RigidPrim("/World/UAVs/UAV_*")
   rp.initialize()
   print(f"Found {rp.count} UAVs")
   print([prim.GetPrimPath() for prim in rp.prims])
   ```

2. **Verify attributes:**
   ```python
   for prim in rp.prims:
       op_id = prim.GetAttribute("NavSim:operator_id").Get()
       uav_id = prim.GetAttribute("NavSim:id").Get()
       print(f"{prim.GetPath()}: operator={op_id}, id={uav_id}")
   ```

3. **Check the bridge log** for "The number of generated flight plans does not match..." errors, which indicate a mismatch between scene UAVs and generated plans.

## Future Enhancements

- [ ] Expose the Viz 08 case selection via Isaac Sim UI instead of environment variable
- [ ] Support heterogeneous operator IDs (multiple operators controlling different UAVs)
- [ ] Persist flight plan selection in a config file instead of env var
