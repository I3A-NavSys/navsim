import numpy as np
from queue import PriorityQueue
import time

class GridNode:
    def __init__(self, i, j, L, s, cost, parent):
        self.i = i
        self.j = j
        self.L = L
        self.s = s
        self.cost = cost
        self.parent = parent

class GridPlanner:

    def __init__(self, cell_side=100, slot_time=10, x_height=60, y_height=100):
        self.cell_side = cell_side  # Tamaño de las celdas           (m)
        self.slot_time = slot_time  # Duración de cada slot          (s)
        self.x_height = x_height    # Altura del subnivel este/oeste (m)
        self.y_height = y_height    # Altura del subnivel norte/sur  (m)
        self.level_height_diff = y_height - x_height
        self.grid = {}              # Diccionario de celdas
        self.is_cost_only = False
        self.respect_limits = True

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
            return [GridNode(i+1, j, 'X', s+2, 0, None), GridNode(i-1, j+1, 'X', s+2, 0, None)]
        else:
            return [GridNode(i+1, j+1, 'X', s+2, 0, None), GridNode(i-1, j, 'X', s+2, 0, None)]
        
    def get_landing_nodes(self, posXY):
        """
        Dada la posición 2D del vertipuerto en el área, 
        devuelve los dos nodos desde los que podemos aterrizar
        """
        (i,j) = np.array(posXY) // self.cell_side
        i = int(i)
        j = int(j)
        if j % 2 == 0:
            return [GridNode(i+1, j+1, 'X', None, 0, None), GridNode(i-1, j, 'X', None, 0, None)]
        else:
            return [GridNode(i-1, j+1, 'X', None, 0, None), GridNode(i+1, j, 'X', None, 0, None)]

    def get_next_node(self, node: GridNode):
        """
        Dado un nodo, devuelve el nodo siguiente en línea recta.
        """
        if node.L == 'X':
            if node.j % 2 == 0:
                # Return None when we get out of the limits of the grid
                if self.respect_limits and node.i + 1 > self.cell_side / self.slot_time:
                    return None

                return GridNode(node.i+1, node.j, 'X', node.s+1, node.cost+1, node)        # rumbo ESTE
            else:
                # Return None when we get out of the limits of the grid
                if self.respect_limits and node.i - 1 < 0:
                    return None
                
                return GridNode(node.i-1, node.j, 'X', node.s+1, node.cost+1, node)        # rumbo OESTE
        
        # L == 'Y'
        else:
            if node.i % 2 == 0:           
                # Return None when we get out of the limits of the grid
                if self.respect_limits and node.j + 1 > self.cell_side / self.slot_time:
                    return None

                return GridNode(node.i, node.j+1, 'Y', node.s+1, node.cost+1, node)        # rumbo NORTE
            else:
                # Return None when we get out of the limits of the grid
                if self.respect_limits and node.j - 1 < 0:
                    return None

                return GridNode(node.i, node.j-1, 'Y', node.s+1, node.cost+1, node)        # rumbo SUR

    def get_cross_node(self, node: GridNode):
        """
        Dado un nodo, devuelve el nodo siguiente en cruce.
        """
        if node.L == 'X':
            if node.j % 2 == 0:                  
                if node.i % 2 == 0:
                    # Return None when we get out of the limits of the grid
                    if self.respect_limits and node.i + 1 > self.cell_side / self.slot_time:
                        return None
                    if self.respect_limits and node.j - 1 < 0:
                        return None

                    return GridNode(node.i+1, node.j-1, 'Y', node.s+1, node.cost+2, node)    # giro ESTE -> SUR
                else:
                    # Return None when we get out of the limits of the grid
                    if self.respect_limits and node.i + 1 > self.cell_side / self.slot_time:
                        return None

                    return GridNode(node.i+1, node.j, 'Y', node.s+1, node.cost+2, node)    # giro ESTE -> NORTE
            else:                           
                if node.i % 2 == 0:              
                    return GridNode(node.i, node.j, 'Y', node.s+1, node.cost+2, node)    # giro OESTE -> NORTE
                else:
                    # Return None when we get out of the limits of the grid
                    if self.respect_limits and node.j - 1 < 0:
                        return None
                    
                    return GridNode(node.i, node.j-1, 'Y', node.s+1, node.cost+2, node)    # giro OESTE -> SUR
        
        # L == 'Y'
        else:
            if node.i % 2 == 0:                  
                if node.j % 2 == 0:
                    # Return None when we get out of the limits of the grid
                    if self.respect_limits and node.i - 1 < 0:
                        return None
                    if self.respect_limits and node.j + 1 > self.cell_side / self.slot_time:
                        return None

                    return GridNode(node.i-1, node.j+1, 'X', node.s+1, node.cost+2, node)    # giro NORTE -> OESTE
                else:
                    # Return None when we get out of the limits of the grid
                    if self.respect_limits and node.j + 1 > self.cell_side / self.slot_time:
                        return None

                    return GridNode(node.i, node.j+1, 'X', node.s+1, node.cost+2, node)    # giro NORTE -> ESTE
            else:                           
                if node.j % 2 == 0:              
                    return GridNode(node.i, node.j, 'X', node.s+1, node.cost+2, node)    # giro SUR -> ESTE
                else:
                    # Return None when we get out of the limits of the grid
                    if self.respect_limits and node.i - 1 < 0:
                        return None

                    return GridNode(node.i-1, node.j, 'X', node.s+1, node.cost+2, node)    # giro SUR -> OESTE

    def get_route(self, start_node: GridNode, end_node: GridNode, cost_only=False, respect_limits=True):
        """
        Dados dos nodos, devuelve una ruta libre del primero al segundo,
        partiendo en el slot especificado.
        """
        start_time = time.time()
        self.is_cost_only = cost_only
        self.respect_limits = respect_limits
        explored_nodes = []
        generation = 0
        prio_queue = PriorityQueue()
        h_start_node = self.evaluate_node(start_node, end_node)
        prio_queue.put((h_start_node, generation, start_node))

        while not prio_queue.empty():
            node: GridNode = prio_queue.get()[2]

            if (node.i, node.j, node.L, node.s) in self.grid:
                continue

            if (node.i, node.j, node.L) == (end_node.i, end_node.j, end_node.L):
                end_time = time.time()
                elapsed_time = end_time - start_time

                return self.get_route_from_node(node), elapsed_time, len(explored_nodes)

            if (node.i, node.j, node.L) in explored_nodes:
                continue
            
            explored_nodes.append((node.i, node.j, node.L))

            next_node = self.get_next_node(node)
            cross_node = self.get_cross_node(node)

            if next_node is not None:
                h_next_node = self.evaluate_node(next_node, end_node)
                generation += 1
                prio_queue.put((h_next_node, generation, next_node))
            
            if cross_node is not None:
                h_cross_node = self.evaluate_node(cross_node, end_node)
                generation += 1
                if not self.is_cost_only:
                    prio_queue.put((h_cross_node+1, generation, cross_node))
                else:
                    prio_queue.put((h_cross_node, generation, cross_node))

        end_time = time.time()
        elapsed_time = end_time - start_time

        return None, elapsed_time, len(explored_nodes)
    
    def get_route_from_node(self, node: GridNode):
        route = []

        while node.parent is not None:
            route.insert(0, node)
            node = node.parent

        route.insert(0, node)
        return route

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
        heuristic = self.get_heuristic_dir(i_diff, j_diff, node, end_node)     

        if not self.is_cost_only:  return heuristic + distance
        else:                   return node.cost + distance + heuristic

    def get_heuristic_dir(self, i_diff, j_diff, node: GridNode, end_node: GridNode):
        heuristic = 0
        
        # Evaluate which direction we should follow and the one we actually are following due to the aeroline we are in
        if node.L == "X":
            # Going right
            if node.j % 2 == 0:
                if i_diff == 0:      heuristic = -1 # Reward been at same level
                elif i_diff > 0:     heuristic = 0  # Correct direction
                else:               heuristic = 1   # Incorrect direction
            
            # Going left
            else:
                if i_diff == 0:      heuristic = -1 # Reward been at same level
                elif i_diff > 0:     heuristic = 1  # Incorrect direction
                else:               heuristic = 0   # Correct direction

        else:
            # Going up
            if node.i % 2 == 0:
                if j_diff == 0:      heuristic = -1 # Reward been at same level
                elif j_diff > 0:     heuristic = 0  # Correct direction
                else:               heuristic = 1   # Incorrect direction
            
            # Going down
            else:
                if j_diff == 0:      heuristic = -1 # Reward been at same level
                elif j_diff > 0:     heuristic = 1  # Incorrect direction
                else:               heuristic = 0   # Correct direction

        # Penalize if we are not arriving to end node from correct direction
        if end_node.L == "X":
            # Arrive from left
            if end_node.j % 2 == 0:
                # Arriving from right
                if i_diff < 0:
                    heuristic += 1

            # Arrive from right
            else:
                # Arriving from left
                if i_diff > 0:
                    heuristic += 1

        else:
            # Arrive from bot
            if end_node.i % 2 == 0:
                # Arriving from top
                if j_diff < 0:
                    heuristic += 1

            # Arrive from top
            else:
                # Arriving from bot
                if j_diff > 0:
                    heuristic += 1

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
        return len(route) - 1


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

    def are_there_conflicts(self, route):
        """
        Comprueba si esta ruta presenta conflictos con rutas existentes.
        :param route: Lista de nodos que forman la ruta.
        """
        for node in route:
            if (node.i, node.j, node.L, node.s) in self.grid:
                return True
        return False