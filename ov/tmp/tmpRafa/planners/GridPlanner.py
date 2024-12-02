import numpy as np
from collections import deque
import time

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
            return [(i+1,j  ,'X',s+2), (i-1,j+1,'X',s+2)]
        else:
            return [(i+1,j+1,'X',s+2), (i-1,j  ,'X',s+2)]
        

    def GetLandingNodes(self, posXY):
        """
        Dada la posición 2D del vertipuerto en el área, 
        devuelve los dos nodos desde los que podemos aterrizar
        """
        (i,j) = np.array(posXY) // self.cell_side
        i = int(i)
        j = int(j)
        if j % 2 == 0:
            return [(i+1,j+1,'X',None), (i-1,j  ,'X',None)]
        else:
            return [(i-1,j+1,'X',None), (i+1,j  ,'X',None)]


    def GetNextNode(self, node):
        """
        Dado un nodo, devuelve el nodo siguiente en línea recta.
        """
        (i,j,L,s) = node
        if L == 'X':
            if j % 2 == 0:                  
                return (i+1,j  ,'X',s+1)        # rumbo ESTE
            else:                           
                return (i-1,j  ,'X',s+1)        # rumbo OESTE
        else: # L == 'Y'
            if i % 2 == 0:                  
                return (i  ,j+1,'Y',s+1)        # rumbo NORTE
            else:                           
                return (i  ,j-1,'Y',s+1)        # rumbo SUR


    def GetCrossNode(self, node):
        """
        Dado un nodo, devuelve el nodo siguiente en cruce.
        """
        (i,j,L,s) = node
        if L == 'X':
            if j % 2 == 0:                  
                if i % 2 == 0:              
                    return (i+1,j-1,'Y',s+1)    # giro ESTE -> SUR
                else:                       
                    return (i+1,j  ,'Y',s+1)    # giro ESTE -> NORTE
            else:                           
                if i % 2 == 0:              
                    return (i  ,j  ,'Y',s+1)    # giro OESTE -> NORTE
                else:                       
                    return (i  ,j-1,'Y',s+1)    # giro OESTE -> SUR
        else: # L == 'Y'
            if i % 2 == 0:                  
                if j % 2 == 0:              
                    return (i-1,j+1,'X',s+1)    # giro NORTE -> OESTE
                else:                       
                    return (i  ,j+1,'X',s+1)    # giro NORTE -> ESTE
            else:                           
                if j % 2 == 0:              
                    return (i  ,j  ,'X',s+1)    # giro SUR -> ESTE
                else:                       
                    return (i-1,j  ,'X',s+1)    # giro SUR -> OESTE


    def GetRoute(self, node1, node2):
        """
        Dados dos nodos, devuelve una ruta libre del primero al segundo,
        partiendo en el slot especificado.
        """
        start_time = time.time()
        (i2,j2,L2,s2) = node2
        # BFS
        explored_nodes = []
        routes = deque()
        routes.append([node1])
        while routes:
            route = routes.popleft()
            node = route[-1]
            (i,j,L,s) = node
            if (i,j,L) in explored_nodes:
                continue
            explored_nodes.append((i,j,L))
            if node in self.grid:
                continue
            if (i,j,L) == (i2,j2,L2):
                end_time = time.time()
                elapsed_time = end_time - start_time
                
                return route, elapsed_time, len(explored_nodes)
            routes.append( route + [self.GetNextNode(node)] )
            routes.append( route + [self.GetCrossNode(node)]  )
        return None
    
    def print_route(self, route):
        print("-- ROUTE -----")
        
        for i in range(len(route)):
            print(f"{i+1}: ({route[i][0]}, {route[i][1]}, {route[i][2]}, {route[i][3]})")
        
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