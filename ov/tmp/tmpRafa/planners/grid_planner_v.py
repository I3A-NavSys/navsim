import numpy as np
from collections import deque
from queue import PriorityQueue
from grid_planner_node import GridPlannerNode
import time

class GridPlanner:

    def __init__(self, cell_side=100, slot_time=10, x_height=60, y_height=100):
        self.cell_side = cell_side  # Tamaño de las celdas           (m)
        self.slot_time = slot_time  # Duración de cada slot          (s)
        self.x_height = x_height    # Altura del subnivel este/oeste (m)
        self.y_height = y_height    # Altura del subnivel norte/sur  (m)
        self.grid = {}              # Diccionario de celdas

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
            return [GridPlannerNode(i+1, j, 'X', s+2, 0, None), GridPlannerNode(i-1, j+1, 'X', s+2, 0, None)]
        else:
            return [GridPlannerNode(i+1, j+1, 'X', s+2, 0, None), GridPlannerNode(i-1, j, 'X', s+2, 0, None)]
        
    def get_landing_nodes(self, posXY):
        """
        Dada la posición 2D del vertipuerto en el área, 
        devuelve los dos nodos desde los que podemos aterrizar
        """
        (i,j) = np.array(posXY) // self.cell_side
        i = int(i)
        j = int(j)
        if j % 2 == 0:
            return [GridPlannerNode(i+1, j+1, 'X', None, 0, None), GridPlannerNode(i-1, j, 'X', None, 0, None)]
        else:
            return [GridPlannerNode(i-1, j+1, 'X', None, 0, None), GridPlannerNode(i+1, j, 'X', None, 0, None)]

    def get_next_node(self, node: GridPlannerNode):
        """
        Dado un nodo, devuelve el nodo siguiente en línea recta.
        """
        if node.L == 'X':
            if node.j % 2 == 0:                  
                return GridPlannerNode(node.i+1, node.j, 'X', node.s+1, node.cost+1, node)        # rumbo ESTE
            else:                           
                return GridPlannerNode(node.i-1, node.j, 'X', node.s+1, node.cost+1, node)        # rumbo OESTE
        
        # L == 'Y'
        else:
            if node.i % 2 == 0:                  
                return GridPlannerNode(node.i, node.j+1, 'Y', node.s+1, node.cost+1, node)        # rumbo NORTE
            else:                           
                return GridPlannerNode(node.i, node.j-1, 'Y', node.s+1, node.cost+1, node)        # rumbo SUR

    def get_cross_node(self, node: GridPlannerNode):
        """
        Dado un nodo, devuelve el nodo siguiente en cruce.
        """
        if node.L == 'X':
            if node.j % 2 == 0:                  
                if node.i % 2 == 0:              
                    return GridPlannerNode(node.i+1, node.j-1, 'Y', node.s+1, node.cost+1, node)    # giro ESTE -> SUR
                else:                       
                    return GridPlannerNode(node.i+1, node.j, 'Y', node.s+1, node.cost+1, node)    # giro ESTE -> NORTE
            else:                           
                if node.i % 2 == 0:              
                    return GridPlannerNode(node.i, node.j, 'Y', node.s+1, node.cost+1, node)    # giro OESTE -> NORTE
                else:                       
                    return GridPlannerNode(node.i, node.j-1, 'Y', node.s+1, node.cost+1, node)    # giro OESTE -> SUR
        
        # L == 'Y'
        else:
            if node.i % 2 == 0:                  
                if node.j % 2 == 0:              
                    return GridPlannerNode(node.i-1, node.j+1, 'X', node.s+1, node.cost+1, node)    # giro NORTE -> OESTE
                else:                       
                    return GridPlannerNode(node.i, node.j+1, 'X', node.s+1, node.cost+1, node)    # giro NORTE -> ESTE
            else:                           
                if node.j % 2 == 0:              
                    return GridPlannerNode(node.i, node.j, 'X', node.s+1, node.cost+1, node)    # giro SUR -> ESTE
                else:                       
                    return GridPlannerNode(node.i-1, node.j, 'X', node.s+1, node.cost+1, node)    # giro SUR -> OESTE

    def get_route(self, start_node: GridPlannerNode, end_node: GridPlannerNode):
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
        # prio_queue.put((0, h_start_node, generation, start_node))

        while not prio_queue.empty():
            node: GridPlannerNode = prio_queue.get()[2]

            if (node.i, node.j, node.L) in explored_nodes:
                continue

            explored_nodes.append((node.i, node.j, node.L))
            
            if (node.i, node.j, node.L, node.s) in self.grid:
                continue

            if (node.i, node.j, node.L) == (end_node.i, end_node.j, end_node.L):
                end_time = time.time()
                elapsed_time = end_time - start_time

                print(f"Search finished")
                print(f"\t-> Elapsed time: {elapsed_time}")
                print(f"\t-> Explored nodes: {len(explored_nodes)}")
                print()

                return self.get_route_from_node(node)

            next_node = self.get_next_node(node)
            h_next_node = self.evaluate_node(next_node, end_node)
            cross_node = self.get_cross_node(node)
            h_cross_node = self.evaluate_node(cross_node, end_node)

            generation += 1
            prio_queue.put((h_next_node, generation, next_node))
            
            generation += 1
            prio_queue.put((h_cross_node, generation, cross_node))

        return None
    
    def get_route_from_node(self, node: GridPlannerNode):
        route = []

        while node.parent is not None:
            route.insert(0, node)
            node = node.parent

        route.insert(0, node)
        return route

    def evaluate_node(self, node: GridPlannerNode, end_node: GridPlannerNode):
        i_diff = end_node.i - node.i
        j_diff = end_node.j - node.j

        heuristic = self.get_heuristic_dir(i_diff, j_diff, node)

        if end_node.L == "X":
            if end_node.j % 2 == 0:
                if i_diff < 0:
                    heuristic += 1

            else:
                if i_diff > 0:
                    heuristic += 1

        else:
            if end_node.i % 2 == 0:
                if j_diff < 0:
                    heuristic += 1

            else:
                if j_diff > 0:
                    heuristic += 1

        distance = np.sqrt(abs(i_diff)**2 + abs(j_diff)**2)

        # jumps = abs(i_diff) + abs(j_diff)

        # return heuristic + jumps
        # return heuristic
        return heuristic + distance


    def get_heuristic_dir(self, i_dir, j_dir, node: GridPlannerNode):
        # if node.L == "X":
        # Going right
        if node.j % 2 == 0:
            if i_dir > 0:   heuristic = 0  # Correct direction
            else:           heuristic = 1  # Incorrect direction
        
        # Going left
        else:
            if i_dir > 0:   heuristic = 1  # Incorrect direction
            else:           heuristic = 0  # Correct direction

        # else:
        # Going up
        if node.i % 2 == 0:
            if j_dir > 0:   heuristic = 0  # Correct direction
            else:           heuristic = 1  # Incorrect direction
        
        # Going down
        else:
            if j_dir > 0:   heuristic = 1  # Incorrect direction
            else:           heuristic = 0  # Correct direction

        return heuristic
    
    def print_route(self, route):
        print("-- ROUTE -----")
        
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
        self.grid[(node2.i, node2.j, node2.L, node2.s)] = 'END'            # END

    def clear_route(self, route):
        """
        Libera la ruta especificada.
        :param route: Lista de nodos que forman la ruta.
        """
        for node in route:
            if node in self.grid:
                del self.grid[node]

    def are_there_conflicts(self, route):
        """
        Comprueba si esta ruta presenta conflictos con rutas existentes.
        :param route: Lista de nodos que forman la ruta.
        """
        for node in route:
            if (node.i, node.j, node.L, node.s) in self.grid:
                return True
        return False