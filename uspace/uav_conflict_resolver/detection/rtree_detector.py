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
        self.tree_index = index.Index(properties=p)
        
        # Dictionary to store all registered UAVs in the system
        # Structure: { uav_id: { "boxes": [SweptBox_OBB, ...], "fp": FlightPlan } }
        # - "boxes": list of swept-volume bounding boxes for the UAV's trajectory
        # - "fp": the FlightPlan object containing waypoints and trajectory data
        self.uavs = {}

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
        
        # Insert each swept box into the 4D R-Tree spatial index for later collision queries
        for i, box in enumerate(boxes):
            # Generate a unique entry ID by combining UAV ID and box index with hash()
            entry_id = hash(f"{uav_id}_{i}")
            
            # Insert into R-Tree with 4D bounds (x_min, y_min, z_min, t_min, x_max, y_max, z_max, t_max)
            # obj parameter stores metadata: (uav_id, box_index) for later reference
            self.tree_index.insert(entry_id, box.get_4d_bounds(), obj=(uav_id, i))

    def _remove_from_index(self, uav_id):
        """
        Remove all swept boxes of a UAV from the 4D R-Tree spatial index.
        This private method is necessary for dynamic updates when flight plans change.
        
        Args:
            uav_id: The unique identifier of the UAV whose boxes should be removed
        """
        # Safety check: ensure the UAV exists in our tracking dictionary
        if uav_id not in self.uavs: 
            return
        
        # Iterate through all swept boxes of this UAV and remove them from the R-Tree
        for i, box in enumerate(self.uavs[uav_id]["boxes"]):
            # Reconstruct the same entry ID used during insertion
            entry_id = hash(f"{uav_id}_{i}")
            
            # Delete the entry from the R-Tree using its ID and 4D bounds
            # This prevents orphaned entries when updating flight plans
            self.tree_index.delete(entry_id, box.get_4d_bounds())

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
            # The objects=True parameter returns associated metadata with each hit
            potential_hits = self.tree_index.intersection(my_box.get_4d_bounds(), objects=True)
            
            # Process each object that passed the broad-phase filter
            for item in potential_hits:
                # Extract the stored metadata: (uav_id, box_index) from the R-Tree entry
                other_uav_id, other_box_idx = item.object
                
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
                potential_hits = self.tree_index.intersection(my_box.get_4d_bounds(), objects=True)
                
                # Process each candidate found by R-Tree
                for item in potential_hits:
                    other_uav_id, other_box_idx = item.object
                    
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
                    if my_box.collides_with(other_box):
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