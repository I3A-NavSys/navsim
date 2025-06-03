import numpy as np
from queue import PriorityQueue
import time
import matplotlib.pyplot as plt

from uspace.flight_plan.flight_plan import FlightPlan

class GridNode:
    def __init__(self, i, j, L, s, cost, parent, state):
        self.i = i
        self.j = j
        self.L = L
        self.s = s
        self.cost = cost
        self.parent = parent
        self.state = state

class GridPlanner:

    def __init__(self, cell_side=100, slot_time=10, x_height=60, y_height=100, max_route_length=100):
        self.cell_side = cell_side  # Tamaño de las celdas           (m)
        self.slot_time = slot_time  # Duración de cada slot          (s)
        self.x_height = x_height    # Altura del subnivel este/oeste (m)
        self.y_height = y_height    # Altura del subnivel norte/sur  (m)
        self.level_height_diff = y_height - x_height
        self.grid = {}              # Diccionario de celdas
        self.max_route_length = max_route_length
        self.debug = False
        self.debug_figure = None

    def get_take_off_nodes(self, posXY, time):
        """
        Dada la posición 2D del vertipuerto en el área, 
        devuelve los dos nodos a los que podemos conectar al despegue
        """
        (i,j) = np.array(posXY) // self.cell_side
        i = int(i)
        j = int(j)
        s = time // self.slot_time
        if j % 2 == 0:
            return [GridNode(i+1, j, 'X', s+2, 0, None, 0), GridNode(i-1, j+1, 'X', s+2, 0, None, 0)]
        else:
            return [GridNode(i+1, j+1, 'X', s+2, 0, None, 0), GridNode(i-1, j, 'X', s+2, 0, None, 0)]
        
    def get_landing_nodes(self, posXY):
        """
        Dada la posición 2D del vertipuerto en el área, 
        devuelve los dos nodos desde los que podemos aterrizar
        """
        (i,j) = np.array(posXY) // self.cell_side
        i = int(i)
        j = int(j)
        if j % 2 == 0:
            return [GridNode(i+1, j+1, 'X', None, 0, None, 0), GridNode(i-1, j, 'X', None, 0, None, 0)]
        else:
            return [GridNode(i-1, j+1, 'X', None, 0, None, 0), GridNode(i+1, j, 'X', None, 0, None, 0)]

    def get_end_node(self, pos, time=0, is_landing=False):
        """
        Given the 2D position of the vertiport using node notation, returns the node to connect to
        """

        (i,j) = pos

        if is_landing:
            if j % 2 == 0:      i -= 1
            else:               i += 1
            s = 0
        
        else:
            if j % 2 == 0:      i += 1
            else:               i -= 1
            s = time + 2

        return GridNode(i, j, 'X', s, 0, None, 0)
    
    def get_next_node(self, node: GridNode):
        """
        Dado un nodo, devuelve el nodo siguiente en línea recta.
        """
        if node.state > 0:      new_state = node.state - 90
        else:                   new_state = 0

        if node.L == 'X':
            if node.j % 2 == 0:
                return GridNode(node.i+1, node.j, 'X', node.s+1, node.cost+1, node, new_state)        # rumbo ESTE
            
            else:
                return GridNode(node.i-1, node.j, 'X', node.s+1, node.cost+1, node, new_state)        # rumbo OESTE
        
        # L == 'Y'
        else:
            if node.i % 2 == 0:
                return GridNode(node.i, node.j+1, 'Y', node.s+1, node.cost+1, node, new_state)        # rumbo NORTE
            
            else:
                return GridNode(node.i, node.j-1, 'Y', node.s+1, node.cost+1, node, new_state)        # rumbo SUR

    def get_cross_node(self, node: GridNode):
        """
        Dado un nodo, devuelve el nodo siguiente en cruce.
        """
        if node.L == 'X':
            if node.j % 2 == 0:                  
                if node.i % 2 == 0:
                    return GridNode(node.i+1, node.j-1, 'Y', node.s+1, node.cost+2, node, node.state + 90)    # giro ESTE -> SUR
                
                else:
                    return GridNode(node.i+1, node.j, 'Y', node.s+1, node.cost+2, node, node.state + 90)    # giro ESTE -> NORTE
            
            else:                           
                if node.i % 2 == 0:              
                    return GridNode(node.i, node.j, 'Y', node.s+1, node.cost+2, node, node.state + 90)    # giro OESTE -> NORTE
                
                else:    
                    return GridNode(node.i, node.j-1, 'Y', node.s+1, node.cost+2, node, node.state + 90)    # giro OESTE -> SUR
        
        # L == 'Y'
        else:
            if node.i % 2 == 0:                  
                if node.j % 2 == 0:
                    return GridNode(node.i-1, node.j+1, 'X', node.s+1, node.cost+2, node, node.state + 90)    # giro NORTE -> OESTE
                
                else:
                    return GridNode(node.i, node.j+1, 'X', node.s+1, node.cost+2, node, node.state + 90)    # giro NORTE -> ESTE
            
            else:                           
                if node.j % 2 == 0:              
                    return GridNode(node.i, node.j, 'X', node.s+1, node.cost+2, node, node.state + 90)    # giro SUR -> ESTE
                
                else:
                    return GridNode(node.i-1, node.j, 'X', node.s+1, node.cost+2, node, node.state + 90)    # giro SUR -> OESTE

    def get_length_from_node(self, node: GridNode):
        if node.parent is None:
            return 1
        return 1 + self.get_length_from_node(node.parent)

    def debug_get_line_from_node(self, node: GridNode):
        x = []
        y = []

        while node.parent is not None:
            if node.L == "X":
                x.append(node.i * 100 + 50)
                y.append(node.j * 100)

            else:
                x.append(node.i * 100)
                y.append(node.j * 100 + 50)

            node = node.parent

        if node.L == "X":
            x.append(node.i * 100 + 50)
            y.append(node.j * 100)

        else:
            x.append(node.i * 100)
            y.append(node.j * 100 + 50)

        return x, y

    def debug_matplotlib(self, node):
        print((node.i, node.j, node.L, node.s, node.state))
        x, y = self.debug_get_line_from_node(node)

        if node.parent is None:
            self.line, = self.debug_figure.plot(x, y, linestyle="dashed", linewidth=1, color="black", zorder=5)
            self.debug_figure.scatter(x, y, color="black", s=10, zorder=5)

        self.line.set_data(x, y)
        self.debug_figure.scatter(x, y, color="black", s=10, zorder=5)

        # plt.draw()
        plt.pause(0.01)

    def get_route(self, start_node: GridNode, end_node: GridNode, are_new_restrictions=True):
        """
        Dados dos nodos, devuelve una ruta libre del primero al segundo,
        partiendo en el slot especificado.
        """
        start_time = time.time()
        explored_nodes = []
        generation = 0
        prio_queue = PriorityQueue()
        h_start_node = self.evaluate_node(start_node, end_node)
        prio_queue.put((h_start_node, generation, start_node))

        while not prio_queue.empty():
            node: GridNode = prio_queue.get()[2]

            if self.debug: self.debug_matplotlib(node)

            # Return if route length exceeds maximum
            if self.get_length_from_node(node) > self.max_route_length:
                continue
            # Continue when node is already reserved
            if (node.i, node.j, node.L, node.s) in self.grid:
                continue

            # Check if current node is end node
            if (node.i, node.j, node.L) == (end_node.i, end_node.j, end_node.L):
                if are_new_restrictions:
                    incorrect_landing_1 = node.state == 90 and node.parent.state == 0
                    incorrect_landing_2 = node.state == 180
                    is_landing_incorrect = incorrect_landing_1 or incorrect_landing_2

                    if is_landing_incorrect:    continue

                    elapsed_time = time.time() - start_time
                    return self.get_route_from_node(node), elapsed_time, len(explored_nodes)
                
                else:
                    end_time = time.time()
                    elapsed_time = end_time - start_time

                    return self.get_route_from_node(node), elapsed_time, len(explored_nodes)

            # Avoid using same node (no time considered) once again
            if (node.i, node.j, node.L) in explored_nodes:
                continue
            
            explored_nodes.append((node.i, node.j, node.L))

            # Expand new nodes according to restrictions
            new_nodes = []
            if are_new_restrictions:
                new_nodes.append(self.get_next_node(node))
                
                is_takeoff_correct = node.parent is not None and node.parent.parent is not None

                if is_takeoff_correct:
                    restricted_maneuver_1 = node.state == 0 and node.parent.state == 90
                    restricted_maneuver_2 = node.state == 90 and node.parent.state == 180
                    restricted_maneuver_3 = node.state == 180
                    include_cross_node = not restricted_maneuver_1 and not restricted_maneuver_2 and not restricted_maneuver_3

                    if include_cross_node:  new_nodes.append(self.get_cross_node(node))
            
            else:
                new_nodes.append(self.get_next_node(node))
                new_nodes.append(self.get_cross_node(node))

            # Evaluate new nodes
            for new_node in new_nodes:
                if new_node is not None:
                    h_new_node = self.evaluate_node(new_node, end_node)
                    generation += 1

                    prio_queue.put((h_new_node + new_node.cost, generation, new_node))

        end_time = time.time()
        elapsed_time = end_time - start_time

        return None, elapsed_time, len(explored_nodes)
    
    def get_best_route(self, option, init_pos, end_pos, init_time, end_time, are_new_restrictions=True):
        """
        Given an init and end time, return the best possible route specified by the option

        Params:
            - option: 
                - 0 for smaller route (least nodes)
                - 1 for the route that reaches the end first in time
                - 2 for the route with the least cost (amount of turns)
            - init_pos:
                - Tuple of grid coordinates for x and y for the initial position
            - end_pos:
                - Tuple of grid coordinates for x and y for the final position
            - init_time:
                - Initial time of the route in time slot
            - end_time: 
                - Final time of the route in time slot
        """

        start_time = time.time()
        routes = []
        prio_length_routes = PriorityQueue()
        prio_time_routes = PriorityQueue()
        prio_cost_routes = PriorityQueue()
        generation = 0
        time_step = self.cell_side/self.slot_time**2
        time_slots_to_search = np.arange(init_time, end_time + time_step, time_step)

        # Iterate though all the time slots
        for time_slot in time_slots_to_search:

            takeoff_node = self.get_end_node(init_pos, int(time_slot), is_landing=False)
            landing_node = self.get_end_node(end_pos, is_landing=True)

            route, elapsed_time, explored_nodes = self.get_route(takeoff_node, landing_node, are_new_restrictions)

            if route is not None:
                generation += 1
                if option == 0:
                    prio_length_routes.put((self.route_length(route), route[-1].s, route[-1].cost, generation, route,
                                            elapsed_time, explored_nodes))
                
                elif option == 1:
                    prio_time_routes.put((route[-1].s, self.route_length(route), route[-1].cost, generation, route,
                                          elapsed_time, explored_nodes))

                elif option == 2:
                    prio_cost_routes.put((route[-1].cost, self.route_length(route), route[-1].s, generation, route,
                                          elapsed_time, explored_nodes))

                routes.append(route)

        final_time = time.time()
        if option == 0 and not prio_length_routes.empty():
            route = prio_length_routes.get()
            return route[4], (route[5], route[6], final_time - start_time)

        elif option == 1 and not prio_time_routes.empty():
            route = prio_time_routes.get()
            return route[4], (route[5], route[6], final_time - start_time)

        elif option == 2 and not prio_cost_routes.empty():
            route = prio_cost_routes.get()
            return route[4], (route[5], route[6], final_time - start_time)

        return None, (None, None, final_time - start_time)

    def get_route_from_node(self, node: GridNode):
        route = []

        while node.parent is not None:
            route.insert(0, node)
            node = node.parent

        route.insert(0, node)
        return route
    
    def get_flightplan_from_route(self, route):
        fp = FlightPlan()
        velocity = self.cell_side / self.slot_time
        offset = self.cell_side / 2

        route_length = len(route)
        node_index = 0
        last_node_index = route_length - 1
        
        while node_index < route_length:
            include = True
            node = route[node_index]

            if node.L == "X":
                pos = [node.i * self.cell_side + offset, node.j * self.cell_side, self.x_height]

                if node.j % 2 == 0:     vel = [velocity, 0, 0]
                else:                   vel = [-velocity, 0, 0]

                # is_change_direction = (last_node_index - node_index >= 2) and (route[node_index + 2].i == node.i) and (abs(route[node_index + 2].j - node.j) == 1)
                is_180_turn = (last_node_index - node_index >= 2) and (route[node_index + 2].i == node.i) and (route[node_index + 2].L == "X")
                is_90_turn = (last_node_index - node_index >= 1) and (route[node_index + 1].L == "Y")
                
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
                pos = [node.i * self.cell_side, node.j * self.cell_side + offset, self.y_height]

                if node.i % 2 == 0:     vel = [0, velocity, 0]
                else:                   vel = [0, -velocity, 0]

                is_180_turn = (last_node_index - node_index >= 2) and (route[node_index + 2].j == node.j) and (route[node_index + 2].L == "Y")
                is_90_turn = (last_node_index - node_index >= 1) and (route[node_index + 1].L == "X")

                if is_180_turn:
                    include = False
                    node_index += 2

                elif is_90_turn:
                    # Check wether we are landing or not
                    if node_index + 1 != last_node_index:
                        node_index += 1

                    include = False

            if include:     fp.set_waypoint(time=node.s * self.slot_time, pos=pos, vel=vel)

            node_index += 1
            
        fp.connect_waypoints()
        return fp

    def evaluate_node(self, node: GridNode, end_node: GridNode):
        # Euclidean distance
        i = node.i
        j = node.j
        end_i = end_node.i
        end_j = end_node.j

        if node.L == "X":       i += 0.5
        else:                   j += 0.5

        if end_node.L == "X":   end_i += 0.5
        else:                   end_j += 0.5

        i_diff = end_i - i
        j_diff = end_j - j

        distance = np.sqrt(abs(i_diff)**2 + abs(j_diff)**2) 

        # Heuristic based on airlines' direction
        heuristic = self.get_heuristic(i_diff, j_diff, node, end_node)

        return heuristic + distance

    def get_heuristic(self, i_diff, j_diff, node: GridNode, end_node: GridNode):
        heuristic = 0

        # Penalize sharp turns and unnecessary deviations
        if abs(i_diff) > 0 and abs(j_diff) > 0:
            if node.parent is not None and node.L != node.parent.L:
                heuristic += 5.263157894736842  # Slight penalty for non-optimal alignments

        # if abs(i_diff) <= 1 or abs(j_diff) <= 1:
        if abs(i_diff) <= 1 and node.parent is not None and node.L != node.parent.L:
                heuristic += 4.7368421052631575  # Slight penalty for non-optimal alignments

        # Evaluate which direction we should follow and the one we actually are following due to the aeroline we are in
        # Going right
        if node.j % 2 == 0 and i_diff < 0:     heuristic += 3.6842105263157894  # Incorrect direction
        # Going left
        elif node.j % 2 != 0 and i_diff > 0:   heuristic += 3.6842105263157894  # Incorrect direction
        
        # Going up
        if node.i % 2 == 0 and j_diff < 0:     heuristic += 3.6842105263157894  # Incorrect direction
        # Going down
        elif node.i % 2 != 0 and j_diff > 0:   heuristic += 3.6842105263157894  # Incorrect direction
        
        
        # Penalize if we are not arriving to end node from correct direction
        # Arrive from left, but Arriving from right
        if end_node.j % 2 == 0 and i_diff < 0:  heuristic += 10
        # Arrive from right, but Arriving from left
        elif end_node.j % 2 != 0 and i_diff > 0:  heuristic += 10

        # Arrive from bot, but Arriving from top
        if end_node.i % 2 == 0 and j_diff < 0:  heuristic += 10
        # Arrive from top, but Arriving from bot
        elif end_node.i % 2 != 0 and j_diff > 0:    heuristic += 10

        return heuristic

    def print_route(self, route):
        print("-- ROUTE -----")
        
        if route is None:
            print("No route could be found. Try a different time slot")
        
        else:
            for i in range(len(route)):
                print(f"{i+1}: ({route[i].i}, {route[i].j}, {route[i].L}, {route[i].s})")
        
        print("--------------")

    def route_length(self, route):
        """
        Dada una ruta, devuelve su longitud.
        """
        return len(route)

    def reserve_nodes(self, route):
        """
        Reserva los nodos que componen la ruta especificada.
        :param route: ruta.
        """
        for idx in range(len(route) - 1):
            node1 = route[idx]
            node2 = route[idx + 1]
            if node1.L == node2.L:
                self.grid[(node1.i, node1.j, node1.L, node1.s)] = 'STR'    # STRaight line
            else:
                self.grid[(node1.i, node1.j, node1.L, node1.s)] = 'TRN'    # TuRN
                
        last_node = route[-1]
        self.grid[(last_node.i, last_node.j, last_node.L, last_node.s)] = 'END'            # END

    def clear_route(self, route):
        """
        Libera la ruta especificada.
        :param route: Lista de nodos que forman la ruta.
        """
        for node in route:
            self.grid.pop((node.i, node.j, node.L, node.s))

    def clear_grid(self):
        self.grid = {}

    def are_there_conflicts(self, route):
        """
        Comprueba si esta ruta presenta conflictos con rutas existentes.
        :param route: Lista de nodos que forman la ruta.
        """
        for node in route:
            if (node.i, node.j, node.L, node.s) in self.grid:
                return True
        return False