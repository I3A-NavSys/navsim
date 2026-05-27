from rtree import index


class RTreeDetector:
    """
    4D R-Tree based conflict detector for UAV flight paths.

    Maintains a spatial-temporal index of all registered UAV trajectories as
    swept-volume OBBs (Oriented Bounding Boxes) and detects conflicts using
    a two-phase pipeline:
      1. BROAD-PHASE : R-Tree intersection query (fast O(log N))
      2. NARROW-PHASE: SAT OBB collision check  (precise)

    This class is an internal engine. External code should interact only
    with CentralManager, which composes RTreeDetector with ConflictResolver.
    """
    def __init__(self):
        """
        Initialize the Strategic Manager with a 4D R-Tree spatial index.
        The R-Tree is configured for 4-dimensional space: X, Y, Z (space), and T (time).
        """
        # Create R-Tree properties and configure for 4D search (X, Y, Z, T coordinates)
        # This allows us to perform spatial-temporal queries efficiently
        p = index.Property()
        p.dimension = 4  # 4D index: X, Y, Z (spatial), T (temporal)
        self._properties = p
        self.tree_index = index.Index(properties=p)
        
        # Dictionary to store all registered UAVs in the system
        # Structure: { uav_id: { "boxes": [SweptBox_OBB, ...], "fp": FlightPlan } }
        # - "boxes": list of swept-volume bounding boxes for the UAV's trajectory
        # - "fp": the FlightPlan object containing waypoints and trajectory data
        self.uavs = {}
        # entry_id -> (uav_id, box_index, bounds)
        self.entry_map = {}
        # uav_id -> [entry_id, ...]
        self.uav_entry_ids = {}
        # Monotonic deterministic id generator for R-Tree entries
        self._next_entry_id = 1

    def _new_entry_id(self):
        entry_id = self._next_entry_id
        self._next_entry_id += 1
        return entry_id

    def _validate_bounds(self, bounds):
        if bounds is None or len(bounds) != 8:
            raise ValueError(f"Invalid 4D bounds length: expected 8 values, got {bounds}")

        if not all(float("-inf") < float(v) < float("inf") for v in bounds):
            raise ValueError(f"Invalid 4D bounds: non-finite values found {bounds}")

        if bounds[0] > bounds[4] or bounds[1] > bounds[5] or bounds[2] > bounds[6] or bounds[3] > bounds[7]:
            raise ValueError(f"Invalid 4D bounds ordering (min > max): {bounds}")

    def _resolve_entry(self, entry_id):
        meta = self.entry_map.get(entry_id)
        if meta is None:
            return None

        other_uav_id, other_box_idx, _ = meta
        other_data = self.uavs.get(other_uav_id)
        if other_data is None:
            return None

        if other_box_idx < 0 or other_box_idx >= len(other_data["boxes"]):
            return None

        return other_uav_id, other_box_idx

    def bulk_register_uavs(self, uav_entries, interval=0.5):
        """
        Bulk-register multiple UAVs in one shot and rebuild the index using STR bulk-load.

        Args:
            uav_entries: iterable of (uav_id, flight_plan)
            interval: interval used to generate OBB swept boxes
        """
        self.uavs = {}
        self.entry_map = {}
        self.uav_entry_ids = {}
        self._next_entry_id = 1

        bulk_items = []
        for uav_id, flight_plan in uav_entries:
            boxes = flight_plan.generate_swept_boxes_obb(interval=interval)
            self.uavs[uav_id] = {"boxes": boxes, "fp": flight_plan}
            self.uav_entry_ids[uav_id] = []

            for i, box in enumerate(boxes):
                bounds = box.get_4d_bounds()
                self._validate_bounds(bounds)
                entry_id = self._new_entry_id()
                # libspatialindex bulk stream expects (id, bounds, obj)
                # We keep obj=None to avoid storing per-entry Python metadata in the index.
                bulk_items.append((entry_id, bounds, None))
                self.entry_map[entry_id] = (uav_id, i, bounds)
                self.uav_entry_ids[uav_id].append(entry_id)

        if bulk_items:
            self.tree_index = index.Index(bulk_items, properties=self._properties)
        else:
            self.tree_index = index.Index(properties=self._properties)

    def register_uav(self, uav_id, flight_plan, interval=0.5):
        """
        Register or update a UAV's flight plan in the central collision detection system.
        This method generates swept-volume bounding boxes and inserts them into the 4D R-Tree.
        
        Args:
            uav_id: Unique identifier for the UAV
            flight_plan: FlightPlan object containing waypoints and trajectory data
            interval: Time interval (seconds) for generating swept boxes along the trajectory
        """
        # If the UAV was already registered, remove its old boxes from the R-Tree index
        # This is necessary for dynamic updates when a flight plan changes.
        # We decided to remove the whole UAV's boxes and re-insert them to ensure consistency, as partial updates can be complex.
        # For now we will KISS (Keep It Simple, Stupid) and just remove all old boxes and re-insert new ones.
        if uav_id in self.uavs:
            self._remove_from_index(uav_id)
        
        # Generate swept-volume bounding boxes using OBB (Oriented Bounding Boxes)
        # These boxes represent the volume swept by the UAV during its flight at regular time intervals
        # OBB is more secure than AABB because it aligns with the UAV's trajectory direction
        boxes = flight_plan.generate_swept_boxes_obb(interval=interval)
        
        # Store the UAV's data in our internal dictionary for quick access
        self.uavs[uav_id] = {"boxes": boxes, "fp": flight_plan}
        self.uav_entry_ids[uav_id] = []
        
        # Insert each swept box into the 4D R-Tree spatial index for later collision queries
        for i, box in enumerate(boxes):
            bounds = box.get_4d_bounds()
            self._validate_bounds(bounds)

            entry_id = self._new_entry_id()
            self.tree_index.insert(entry_id, bounds)
            self.entry_map[entry_id] = (uav_id, i, bounds)
            self.uav_entry_ids[uav_id].append(entry_id)

    def _remove_from_index(self, uav_id):
        """
        Remove all swept boxes of a UAV from the 4D R-Tree spatial index.
        This private method is necessary for dynamic updates when flight plans change.
        
        Args:
            uav_id: The unique identifier of the UAV whose boxes should be removed
        """
        # Safety check: ensure the UAV exists in our tracking dictionary
        entry_ids = self.uav_entry_ids.get(uav_id, [])
        for entry_id in entry_ids:
            meta = self.entry_map.get(entry_id)
            if meta is None:
                continue
            _, _, bounds = meta
            self.tree_index.delete(entry_id, bounds)
            del self.entry_map[entry_id]

        self.uav_entry_ids[uav_id] = []

    def detect_all_conflicts(self, target_uav_id):
        """
        Detect all conflicts for a specific UAV against all other aircraft in the system.
        Uses a two-stage collision detection algorithm:
        1. BROAD-PHASE: R-Tree spatial queries to find potential collisions (fast)
        2. NARROW-PHASE: Precise SAT (Separating Axis Theorem) OBB collision tests (accurate)
        
        This hybrid approach balances performance and accuracy by avoiding unnecessary
        precision checks on spatially separated objects.
        
        Args:
            target_uav_id: The UAV to check for conflicts
            
        Returns:
            List of conflict dictionaries containing involved UAVs and collision details

        We will use this function for targeted conflict detection, which is more efficient when
          we only care about one UAV's safety. Very suitable for conflict resolution.
                    
        For a complete system-wide check, we will use detect_all_conflicts_system_wide() instead.
        """
        # Initialize empty list to store detected conflicts
        conflicts = []
        
        # Safety check: ensure the target UAV is registered in the system
        if target_uav_id not in self.uavs: 
            return conflicts
        
        # Retrieve all swept boxes of the target UAV
        target_boxes = self.uavs[target_uav_id]["boxes"]
        
        # Iterate through each swept box of the target UAV
        for i, my_box in enumerate(target_boxes):
            # ========== PHASE 1: BROAD-PHASE FILTERING ==========
            # Query the R-Tree to find all boxes that spatially overlap with my_box
            # in 4D space (x, y, z, time). This is very fast due to R-Tree efficiency.
            potential_hits = self.tree_index.intersection(my_box.get_4d_bounds())
            
            # Process each object that passed the broad-phase filter
            for entry_id in potential_hits:
                resolved = self._resolve_entry(entry_id)
                if resolved is None:
                    continue
                other_uav_id, other_box_idx = resolved
                
                # Skip self-collision: don't check a UAV against itself
                if other_uav_id == target_uav_id: 
                    continue
                
                # ========== PHASE 2: NARROW-PHASE PRECISE TEST ==========
                # Retrieve the other UAV's swept box from our tracking dictionary
                other_box = self.uavs[other_uav_id]["boxes"][other_box_idx]
                
                # Perform precise collision detection using SAT (Separating Axis Theorem)
                # SAT checks if two OBBs truly overlap by testing projection on all axes
                # This is more computationally expensive but very accurate

                is_collision, mtv_vector = my_box.collides_with(other_box)
                
                if is_collision:
                    # A real collision was detected! Record the conflict details
                    conflicts.append({
                        "uav_a": target_uav_id,           # First UAV involved
                        "uav_b": other_uav_id,             # Second UAV involved
                        "box_a_idx": i,                    # Index of first UAV's swept box
                        "box_b_idx": other_box_idx,        # Index of second UAV's swept box
                        "time_range": my_box.t_range,       # Temporal interval of collision
                        "mtv": mtv_vector
                    })
        
        return conflicts

    def detect_candidate_conflicts(self, candidate_boxes, candidate_uav_id=None, early_exit: bool = True):
        """
        Detect conflicts between a set of candidate OBBs and the currently indexed UAVs
        WITHOUT inserting the candidate boxes into the R-Tree.

        This is intended for validating a proposed route/strategy quickly: for each
        candidate swept box we query the existing R-Tree for potential overlaps and
        run the narrow-phase SAT check only on hits. Optionally returns immediately on
        the first detected conflict (`early_exit=True`).

        Args:
            candidate_boxes: iterable/list of SweptBox_OBB for the candidate route
            candidate_uav_id: optional id of the candidate UAV (used to skip self-hits)
            early_exit: if True return as soon as any conflict is found

        Returns:
            List of conflict dicts (same format as `detect_all_conflicts`) — empty if none.
        """
        conflicts = []

        for i, my_box in enumerate(candidate_boxes):
            potential_hits = self.tree_index.intersection(my_box.get_4d_bounds())

            for entry_id in potential_hits:
                resolved = self._resolve_entry(entry_id)
                if resolved is None:
                    continue
                other_uav_id, other_box_idx = resolved

                # Skip if the hit corresponds to the same UAV (when candidate is an update)
                if candidate_uav_id is not None and str(other_uav_id) == str(candidate_uav_id):
                    continue

                # Narrow-phase: precise SAT collision test
                other_box = self.uavs[other_uav_id]["boxes"][other_box_idx]
                is_collision, mtv_vector = my_box.collides_with(other_box)

                if is_collision:
                    conflicts.append({
                        "uav_a": candidate_uav_id if candidate_uav_id is not None else "candidate",
                        "uav_b": other_uav_id,
                        "box_a_idx": i,
                        "box_b_idx": other_box_idx,
                        "time_range": my_box.t_range,
                        "mtv": mtv_vector,
                    })

                    if early_exit:
                        return conflicts

        return conflicts
    
    def detect_all_conflicts_system_wide(self):
        """
        Detect ALL conflicts in the system across all registered UAVs without duplicates.
        Uses the 4D R-Tree to maintain high performance (Broad-phase -> Narrow-phase).

        We will use this function for a complete system-wide conflict detection, which is more expensive 
        but necessary for comprehensive safety checks.
        """
        conflicts = []
        uav_ids = list(self.uavs.keys())
        
        # Map UAV ID to its index for fast duplicate detection
        uav_index_map = {uav_id: idx for idx, uav_id in enumerate(uav_ids)}
        
        # Iterate through each UAV and its boxes
        for current_uav_id, current_data in self.uavs.items():
            current_uav_idx = uav_index_map[current_uav_id]
            current_boxes = current_data["boxes"]
            
            # Process each box of the current UAV
            for box_idx, my_box in enumerate(current_boxes):
                # ========== PHASE 1: BROAD-PHASE FILTERING ==========
                # Use R-Tree to find only nearby boxes in space-time (efficient O(log N))
                potential_hits = self.tree_index.intersection(my_box.get_4d_bounds())
                
                # Process each candidate found by R-Tree
                for entry_id in potential_hits:
                    resolved = self._resolve_entry(entry_id)
                    if resolved is None:
                        continue
                    other_uav_id, other_box_idx = resolved
                    
                    # Skip self-collisions
                    if other_uav_id == current_uav_id:
                        continue
                    
                    # ========== DUPLICATE AVOIDANCE ==========
                    # Process each pair (A,B) exactly once by index comparison:
                    # Only process if other_uav_index > current_uav_index
                    # If other_uav_index <= current_uav_index, pair was already processed
                    other_uav_idx = uav_index_map[other_uav_id]
                    if other_uav_idx <= current_uav_idx:
                        continue
                    
                    # ========== PHASE 2: NARROW-PHASE PRECISE TEST ==========
                    other_box = self.uavs[other_uav_id]["boxes"][other_box_idx]
                    
                    # Perform precise SAT collision detection
                    is_collision, _ = my_box.collides_with(other_box)
                    if is_collision:
                        conflicts.append({
                            "uav_a": current_uav_id,
                            "uav_b": other_uav_id,
                            "box_a_idx": box_idx,
                            "box_b_idx": other_box_idx,
                            "time_range": my_box.t_range
                        })
        
        return conflicts
    
    # We managed to detect conflicts efficiently by using R-Tree for broad-phase filtering and SAT for narrow-phase.
    # The system-wide detection automatically avoids duplicates by iterating unique pairs only.