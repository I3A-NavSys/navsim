import numpy as np
from collections import deque
from queue import PriorityQueue
from grid_planner_node import GridPlannerNode

class GridPlanner:

    def __init__(self, cell_side=100, slot_time=10, x_height=60, y_height=100):
        self.cell_side = cell_side  # Tamaño de las celdas           (m)
        self.slot_time = slot_time  # Duración de cada slot          (s)
        self.x_height = x_height    # Altura del subnivel este/oeste (m)
        self.y_height = y_height    # Altura del subnivel norte/sur  (m)
        self.grid = {}              # Diccionario de celdas

    def GetTakeOffNodes(self, posXY, time):
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
        
    def GetLandingNodes(self, posXY):
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

    def GetNextNode(self, node: GridPlannerNode):
        """
        Dado un nodo, devuelve el nodo siguiente en línea recta.
        """
        if node.L == 'X':
            if node.j % 2 == 0:                  
                return GridPlannerNode(node.i+1, node.j, 'X', node.s+1, 0, node)        # rumbo ESTE
            else:                           
                return GridPlannerNode(node.i-1, node.j, 'X', node.s+1, 0, node)        # rumbo OESTE
        
        # L == 'Y'
        else:
            if node.i % 2 == 0:                  
                return GridPlannerNode(node.i, node.j+1, 'Y', node.s+1, 0, node)        # rumbo NORTE
            else:                           
                return GridPlannerNode(node.i, node.j-1, 'Y', node.s+1, 0, node)        # rumbo SUR

    def GetCrossNode(self, node: GridPlannerNode):
        """
        Dado un nodo, devuelve el nodo siguiente en cruce.
        """
        if node.L == 'X':
            if node.j % 2 == 0:                  
                if node.i % 2 == 0:              
                    return GridPlannerNode(node.i+1, node.j-1, 'Y', node.s+1, 0, node)    # giro ESTE -> SUR
                else:                       
                    return GridPlannerNode(node.i+1, node.j, 'Y', node.s+1, 0, node)    # giro ESTE -> NORTE
            else:                           
                if node.i % 2 == 0:              
                    return GridPlannerNode(node.i, node.j, 'Y', node.s+1, 0, node)    # giro OESTE -> NORTE
                else:                       
                    return GridPlannerNode(node.i, node.j-1, 'Y', node.s+1, 0, node)    # giro OESTE -> SUR
        
        # L == 'Y'
        else:
            if node.i % 2 == 0:                  
                if node.j % 2 == 0:              
                    return GridPlannerNode(node.i-1, node.j+1, 'X', node.s+1, 0, node)    # giro NORTE -> OESTE
                else:                       
                    return GridPlannerNode(node.i, node.j+1, 'X', node.s+1, 0, node)    # giro NORTE -> ESTE
            else:                           
                if node.j % 2 == 0:              
                    return GridPlannerNode(node.i, node.j, 'X', node.s+1, 0, node)    # giro SUR -> ESTE
                else:                       
                    return GridPlannerNode(node.i-1, node.j, 'X', node.s+1, 0, node)    # giro SUR -> OESTE

    def GetRoute(self, start_node: GridPlannerNode, end_node: GridPlannerNode):
        """
        Dados dos nodos, devuelve una ruta libre del primero al segundo,
        partiendo en el slot especificado.
        """

        explored_nodes = []
        generation = 0
        prio_queue = PriorityQueue()
        eval_cost = self.evaluate_node(start_node, end_node)
        prio_queue.put((eval_cost, generation, start_node))

        while not prio_queue.empty():
            node: GridPlannerNode = prio_queue.get()[2]

            if (node.i, node.j, node.L) in explored_nodes:
                continue

            explored_nodes.append((node.i, node.j, node.L))
            
            if node in self.grid:
                continue

            if (node.i, node.j, node.L) == (end_node.i, end_node.j, end_node.L):
                print(f"Search finished -> Explored nodes: {len(explored_nodes)}")
                return self.get_route_from_node(node)

            next_node = self.GetNextNode(node)
            next_node_eval_cost = self.evaluate_node(next_node, end_node)
            cross_node = self.GetCrossNode(node)
            cross_node_eval_cost = self.evaluate_node(cross_node, end_node)

            generation += 1
            prio_queue.put((next_node_eval_cost, generation, next_node))
            
            generation += 1
            prio_queue.put((cross_node_eval_cost, generation, cross_node))

        return None
    
    def get_route_from_node(self, node: GridPlannerNode):
        route = []

        while node.parent is not None:
            route.insert(0, node)
            node = node.parent

        route.insert(0, node)
        return route

    def evaluate_node(self, node: GridPlannerNode, end_node: GridPlannerNode):
        # Heuristic function
        i_dir = end_node.i - node.i
        j_dir = end_node.j - node.j

        hi_dir, hj_dir = self.get_heuristic_dir(i_dir, j_dir, node)

        # Cost function (manhattan distance)
        jumps_cost = abs(i_dir) + abs(j_dir)

        return hi_dir + hj_dir + jumps_cost


    def get_heuristic_dir(self, i_dir, j_dir, node: GridPlannerNode):        
        # if L == "X": This is for better implementation

        # Going right
        if node.i % 2 == 0:
            if i_dir > 0:   hi_dir = 0  # Correct direction
            else:           hi_dir = 1  # Incorrect direction
        
        # Going left
        else:
            if i_dir > 0:   hi_dir = 1  # Incorrect direction
            else:           hi_dir = 0  # Correct direction

        # Going up
        if node.j % 2 == 0:
            if j_dir > 0:   hj_dir = 0  # Correct direction
            else:           hj_dir = 1  # Incorrect direction
        
        # Going down
        else:
            if j_dir > 0:   hj_dir = 1  # Incorrect direction
            else:           hj_dir = 0  # Correct direction

        return hi_dir, hj_dir
    
    def print_route(self, route):
        print("-- ROUTE -----")
        
        for i in range(len(route)):
            print(f"{i+1}: ({route[i].i}, {route[i].j}, {route[i].L}, {route[i].s})")
        
        print("--------------")

    def RouteLength(self, route):
        """
        Dada una ruta, devuelve su longitud.
        """
        return len(route) - 1


    def ReserveNodes(self, route):
        """
        Reserva los nodos que componen la ruta especificada.
        :param route: ruta.
        """
        for idx in range(len(route) - 1):
            node1 = route[idx]
            node2 = route[idx + 1]
            if node1[2] == node2[2]:
                self.grid[node1] = 'STR'    # STRaight line
            else:
                self.grid[node1] = 'TRN'    # TuRN
        self.grid[node2] = 'END'            # END


    def ClearRoute(self, route):
        """
        Libera la ruta especificada.
        :param route: Lista de nodos que forman la ruta.
        """
        for node in route:
            if node in self.grid:
                del self.grid[node]

    def AreThereConflicts(self, route):
        """
        Comprueba si esta ruta presenta conflictos con rutas existentes.
        :param route: Lista de nodos que forman la ruta.
        """
        for node in route:
            if node in self.grid:
                return True
        return False