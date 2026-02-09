import heapq
import matplotlib.pyplot as plt
import math


from uspace.flight_plan.flight_plan import FlightPlan


class GridNode:
    def __init__(self, i, j, l, s, cost, parent, state):
        self.i = i
        self.j = j
        self.l = l
        self.s = s
        self.cost = cost
        self.parent = parent
        self.state = state

    def __lt__(self, other):
        return self.cost < other.cost

class GridPlanner:

    def __init__(self, cell_side=100, slot_time=10, x_height=60, y_height=100, max_route_length=100):
        self.cell_side = cell_side  # Size of the cells                     (m)
        self.slot_time = slot_time  # Duration of each slot                 (s)
        self.x_height = x_height    # Height of the east/west sublevel      (m)
        self.y_height = y_height    # Height of the north/south sublevel    (m)
        self.level_height_diff = y_height - x_height
        self.grid = set()           # Set of reserved nodes
        self.max_route_length = max_route_length
        self.debug = False
        self.debug_figure = None


    # -------------------------
    # --- Debug Methods -------
    # -------------------------
    def debug_get_line_from_node(self, node: GridNode):
        x = []
        y = []
        additional_offset = self.cell_side
        half_addional_offset = additional_offset / 2

        while node.parent is not None:
            if node.l == "X":
                x.append(node.i * additional_offset + half_addional_offset)
                y.append(node.j * additional_offset)

            else:
                x.append(node.i * additional_offset)
                y.append(node.j * additional_offset + half_addional_offset)

            node = node.parent

        if node.l == "X":
            x.append(node.i * additional_offset + half_addional_offset)
            y.append(node.j * additional_offset)

        else:
            x.append(node.i * additional_offset)
            y.append(node.j * additional_offset + half_addional_offset)

        return x, y

    def debug_matplotlib(self, node: GridNode):
        print((node.i, node.j, node.l, node.s, node.state))
        x, y = self.debug_get_line_from_node(node)

        if node.parent is None:
            self.line, = self.debug_figure.plot(x, y, linestyle="dashed", linewidth=1, color="black", zorder=5)
            self.debug_figure.scatter(x, y, color="black", s=10, zorder=5)

        self.line.set_data(x, y)
        self.debug_figure.scatter(x, y, color="black", s=10, zorder=5)

        plt.pause(0.01)


    # -------------------------
    # --- Auxiliary Methods ---
    # -------------------------
    def build_node_from_coords(
        self, 
        coords: list[float], 
        time: float, 
        cost: int, 
        parent: GridNode, 
        state: int
    ):
        """
        Returns the grid node corresponding to the given Cartesian coordinates.
        
        :param coords: Cartesian coordinates [x, y, z]
        :param time: Time in seconds
        :param cost: Cost to reach this node
        :param parent: Parent node
        :param state: State of the node
        """

        x, y, z = coords
        i = int(x // self.cell_side)
        j = int(y // self.cell_side)
        s = math.ceil(time / self.slot_time)

        # if z <= self.x_height:
        #     l = 'X'
        # else:
        #     l = 'Y'
        l = "X"

        return GridNode(i, j, l, s, cost, parent, state)

    def get_length_from_node(self, node: GridNode):
        """
        Given a node, returns the length from the start node to this node.
        
        :param node: Node to get length from
        """

        length = 0

        while node.parent is not None:
            length += 1
            node = node.parent

        return length + 1

    def get_route_from_node(self, node: GridNode, reverse: bool):
        """
        Given a node, returns the correspoding route composed by itself and its parents.
        
        :param node: Node to build route from
        :param reverse: Whether to reverse the route order
        """

        route = []
        insertion = lambda x: route.insert(0, x)
        if reverse:
            insertion = route.append

        while node.parent is not None:
            insertion((node.i, node.j, node.l, node.s))
            node = node.parent

        # Final node
        insertion((node.i, node.j, node.l, node.s))
        return route
    
    def get_flightplan_from_route(self, route: list[(int, int, str, int)]):
        fp = FlightPlan()
        velocity = self.cell_side / self.slot_time
        offset = self.cell_side / 2

        route_length = len(route)
        node_index = 0
        last_node_index = route_length - 1
        
        while node_index < route_length:
            include = True
            i, j, l, s = route[node_index]

            if l == "X":
                pos = [i * self.cell_side + offset, j * self.cell_side, self.x_height]

                if j % 2 == 0:  vel = [velocity, 0, 0]
                else:           vel = [-velocity, 0, 0]

                is_180_turn = (
                    last_node_index - node_index >= 2   and 
                    route[node_index + 2][0] == i        and 
                    route[node_index + 2][2] == "X"
                )
                is_90_turn = (
                    last_node_index - node_index >= 1   and 
                    route[node_index + 1][2] == "Y"
                )
                
                if is_180_turn:
                    if node_index != 0:
                        include = False
                    
                    node_index += 2

                    if node_index == last_node_index:
                        node_index -= 1

                elif is_90_turn:
                    # Check wether we are taking off or not
                    if node_index != 0:     
                        include = False

                    node_index += 1

            else:
                pos = [i * self.cell_side, j * self.cell_side + offset, self.y_height]

                if i % 2 == 0:  vel = [0, velocity, 0]
                else:           vel = [0, -velocity, 0]

                is_180_turn = (
                    last_node_index - node_index >= 2   and 
                    route[node_index + 2][1] == j        and 
                    route[node_index + 2][2] == "Y"
                )
                is_90_turn = (
                    last_node_index - node_index >= 1   and 
                    route[node_index + 1][2] == "X"
                )

                if is_180_turn:
                    include = False
                    node_index += 2

                elif is_90_turn:
                    # Check wether we are landing or not
                    if node_index + 1 != last_node_index:
                        node_index += 1

                    include = False

            if include:
                fp.set_waypoint(time=s * self.slot_time, pos=pos, vel=vel)

            node_index += 1
            
        fp.connect_waypoints()
        return fp

    def print_route(self, route: list[(int, int, str, int)]):
        print("-- ROUTE -----")
        
        if not route:
            print("No route could be found. Try a different time slot")
        else:
            for id, n in enumerate(route):
                i, j, l, s = n
                print(f"{id+1}: ({i}, {j}, {l}, {s})")
        
        print("--------------")

    def reserve_route(self, route: list[(int, int, str, int)]):
        """
        Reserve the nodes that composed the specified route.

        :param route: List of nodes that compose the route.
        """

        for (i, j, l, s) in route:
            self.grid.add((i, j, l, s))

    def free_route(self, route: list[(int, int, str, int)]):
        """
        Free the nodes that composed the specified route.

        :param route: List of nodes that compose the route.
        """

        for (i, j, l, s) in route:
            self.grid.remove((i, j, l, s))

    def clear_grid(self):
        self.grid.clear()

    def are_there_conflicts(self, route: list[(int, int, str, int)]):
        """
        Check if there are conflicts in the specified route.
        
        :param route: List of nodes that compose the route.
        """

        for (i, j, l, s) in route:
            if (i, j, l, s) in self.grid:
                return True
        return False


    # -------------------------
    # --- Main Methods --------
    # -------------------------
    def get_next_node(self, node: GridNode, reverse: bool):
        """
        Given a node, returns the next node in the same line.

        :param node: Current grid node
        :param reverse: Whether to go in reverse direction
        """

        direction = 1
        if reverse:
            direction = -1

        if node.state > 0:      new_state = node.state - 90
        else:                   new_state = 0

        if node.l == 'X':
            if node.j % 2 == 0:
                # EAST course
                i = node.i + direction
                j = node.j
                l = 'X'
            else:
                # WEST course
                i = node.i - direction
                j = node.j
                l = 'X'
        else:
            if node.i % 2 == 0:
                # NORTH course
                i = node.i
                j = node.j + direction
                l = 'Y'
            else:
                # SOUTH course
                i = node.i
                j = node.j - direction
                l = 'Y'
        
        return GridNode(i, j, l, node.s + direction, node.cost + 1, node, new_state)

    def get_cross_node(self, node: GridNode, reverse: bool):
        """
        Given a node, returns the crossing node.

        :param node: Current grid node
        :param reverse: Whether to go in reverse direction
        """

        revert = int(reverse)
        s_increment = 1 - revert * 2

        if node.l == 'X':
            if node.j % 2 == 0:
                # EAST course
                if node.i % 2 == 0:
                    # EAST to SOUTH turn
                    i = node.i + 1 - revert
                    j = node.j - 1
                    l = 'Y'
                
                else:
                    # EAST to NORTH turn
                    i = node.i + 1 - revert
                    j = node.j
                    l = 'Y'
            else:
                # WEST course
                if node.i % 2 == 0:
                    # WEST to NORTH turn
                    i = node.i + revert
                    j = node.j
                    l = 'Y'
                else:
                    # WEST to SOUTH turn
                    i = node.i + revert
                    j = node.j - 1
                    l = 'Y'
        else:
            if node.i % 2 == 0:
                # NORTH course
                if node.j % 2 == 0:
                    # NORTH to WEST turn
                    i = node.i - 1
                    j = node.j + 1 - revert
                    l = 'X'
                else:
                    # NORTH to EAST turn
                    i = node.i
                    j = node.j + 1 - revert
                    l = 'X'
            else:
                # SOUTH course
                if node.j % 2 == 0:
                    # SOUTH to EAST turn
                    i = node.i
                    j = node.j + revert
                    l = 'X'
                else:
                    # SOUTH to WEST turn
                    i = node.i - 1
                    j = node.j + revert
                    l = 'X'
        
        return GridNode(i, j, l, node.s + s_increment, node.cost + 2, node, node.state + 90)

    def evaluate_node(self, node: GridNode, end_node: GridNode, reverse: bool):
        """
        Given a node and the end node, returns its f score for A* evaluation.

        :param node: Current grid node
        :param end_node: Target grid node
        :param reverse: Whether to go in reverse direction
        """

        # Set each node at its correct 3D position
        current_i = node.i
        current_j = node.j
        end_i = end_node.i
        end_j = end_node.j
        offset = 0.5
        h_weight = 2

        if node.l == "X":       current_i += offset
        else:                   current_j += offset

        if end_node.l == "X":   end_i += offset
        else:                   end_j += offset

        dx = end_i - current_i
        dy = end_j - current_j
        
        # Compute heuristic score
        h_score = self.get_heuristic(dx, dy, node, reverse)
        # Multiply by 1.001 to avoid ties in the priority queue
        h_score *= 1.001

        f_score = node.cost + h_score * h_weight
        return f_score

    def get_heuristic(self, dx, dy, node: GridNode, reverse: bool):
        """
        Returns the heuristic score for the given node.

        :param dx: Distance in X to the goal
        :param dy: Distance in Y to the goal
        :param node: Current grid node
        :param reverse: Whether to go in reverse direction
        """
        
        # --- 1. Base Distance (Manhattan) ---
        h = abs(dx) + abs(dy)

        # --- 2. Penalty for layer change ---
        # If there is distance in both X and Y, there will necessarily be a turn (+2 cost)
        # Manhattan assumes a cost of 1 per cell
        # The turn adds +1 extra net
        if dx != 0 and dy != 0:
            h += 1

        # --- 3. Penalization for Direction of the Airway ---
        # Here we determine if the current node is on an airway that takes us away from 
        # the goal
        wrong_direction = False
        
        if node.l == 'X':
            # Normal rule: Evens go East (+1), Odds go West (-1)
            grid_flow = 1 if node.j % 2 == 0 else -1
            
            # If we are in reverse mode, we navigate against the flow, 
            # so the effective flow to reach the previous node is the opposite.
            if reverse:
                grid_flow *= -1
            
            # Evaluation:
            # If I want to go East (dx > 0) but the flow is West (-1) -> Bad
            if dx > 0 and grid_flow == -1: wrong_direction = True
            elif dx < 0 and grid_flow == 1: wrong_direction = True

        elif node.l == 'Y':
            # Normal rule: Evens go North (+1), Odds go South (-1)
            grid_flow = 1 if node.i % 2 == 0 else -1
            
            if reverse:
                grid_flow *= -1

            # Evaluation:
            # If I want to go North (dy > 0) but the flow is South (-1) -> Bad
            if dy > 0 and grid_flow == -1: wrong_direction = True
            elif dy < 0 and grid_flow == 1: wrong_direction = True

        if wrong_direction:
            # If we go against the flow, we have to turn, move parallel, and turn again.
            # This adds at least 2 extra steps of cost over Manhattan.
            h += 2

        return h

    def get_route(
            self, 
            origin: list[float, float, float], 
            destination: list[float, float, float], 
            start_time: float,
            end_time: float,
            reverse: bool=False
    ):
        """
        Given an origin and destination, returns the best route between both points.

        :param origin: Cartesian coordinates of the origin [x, y, z]
        :param destination: Cartesian coordinates of the destination [x, y, z]
        :param start_time: Start time in seconds
        :param end_time: End time in seconds
        :param reverse: Whether to compute the route in reverse direction
        """

        # Variables initialization
        start_node = self.build_node_from_coords(origin, start_time, 0, None, 0)
        end_node = self.build_node_from_coords(destination, end_time, 0, None, 0)
        explored_nodes = []
        generation = 0
        open_nodes = []
        h_start_node = self.evaluate_node(start_node, end_node, reverse)
        heapq.heappush(open_nodes, (h_start_node, generation, start_node))

        # Main loop
        while open_nodes:
            # Get node with highest priority
            queue_item = heapq.heappop(open_nodes)
            node_generation = queue_item[1]
            node: GridNode = queue_item[2]

            if self.debug: self.debug_matplotlib(node)

            # Define conditions to continue or return route
            is_route_length_exceeded = node_generation > self.max_route_length
            is_node_reserved = (node.i, node.j, node.l, node.s) in self.grid
            is_node_explored = (node.i, node.j, node.l) in explored_nodes
            is_end_node = (node.i, node.j, node.l) == (end_node.i, end_node.j, end_node.l)

            if is_route_length_exceeded or is_node_reserved or is_node_explored:
                continue

            # Evaluate end condition
            if is_end_node:
                incorrect_landing_1 = node.state == 90 and node.parent.state == 0
                incorrect_landing_2 = node.state == 180
                is_landing_incorrect = incorrect_landing_1 or incorrect_landing_2

                if is_landing_incorrect:    continue

                return self.get_route_from_node(node, reverse)
            
            # Mark node as explored
            explored_nodes.append((node.i, node.j, node.l))

            # Expand new nodes
            new_nodes = [self.get_next_node(node, reverse)]
            # is_takeoff_correct = node.parent is not None and node.parent.parent is not None
            is_takeoff_correct = node.parent is not None

            if is_takeoff_correct:
                restricted_maneuver_1 = node.state == 0 and node.parent.state == 90
                restricted_maneuver_2 = node.state == 90 and node.parent.state == 180
                restricted_maneuver_3 = node.state == 180
                include_cross_node = (
                    not restricted_maneuver_1   and 
                    not restricted_maneuver_2   and 
                    not restricted_maneuver_3
                )

                if include_cross_node:
                    new_nodes.append(self.get_cross_node(node, reverse))
            
            # Evaluate new nodes
            for new_node in new_nodes:
                f_new_node = self.evaluate_node(new_node, end_node, reverse)
                generation += 1

                heapq.heappush(open_nodes, (f_new_node, generation, new_node))

        return []
    