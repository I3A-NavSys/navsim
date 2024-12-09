import sys
import os
import random
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import numpy as np

#borrar consola
os.system('cls')

# Añadir el directorio principal al sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

##############################################################################


from uspace.grid_planner.grid_planner import GridPlanner as gp_A
from tmp.tmpRafa.planners.GridPlanner import GridPlanner as gp_NA


gp_Astar = gp_A()
gp_no_Astar = gp_NA()

TOpos = (-3000, -3000)  # posición de despegue   (m)
TOtime = 0         # tiempo de despegue     (s)
Lpos  = (3000, 3000)  # posición de aterrizaje (m)

TOnodes_rafa = gp_no_Astar.GetTakeOffNodes(TOpos,TOtime)
Lnodes_rafa  = gp_no_Astar.GetLandingNodes(Lpos)

TOnodes_victor = gp_Astar.get_take_off_nodes(TOpos,TOtime)
Lnodes_victor  = gp_Astar.get_landing_nodes(Lpos)

# Route 1
print("###################################")
print("COMPUTING ROUTE")
print(f"Origin: {TOnodes_victor[0].i, TOnodes_victor[0].j, TOnodes_victor[0].L, TOnodes_victor[0].s}")
print(f"Destination: {Lnodes_victor[0].i, Lnodes_victor[0].j, Lnodes_victor[0].L, Lnodes_victor[0].s}")
print()

route, e_time, explored_nodes  = gp_no_Astar.GetRoute(TOnodes_rafa[0], Lnodes_rafa[0])
print("WITHOUT A*")
gp_no_Astar.print_route(route)
conflicts = gp_no_Astar.AreThereConflicts(route)
length = gp_no_Astar.RouteLength(route)

print(f"Conflicts: {conflicts}")
print(f"Length: {length}")
print(f"Time: {e_time}")
print(f"Explored nodes: {explored_nodes}")
print()

print("WITH A*")
route, e_time, explored_nodes = gp_Astar.get_route(TOnodes_victor[0], Lnodes_victor[0])
gp_Astar.print_route(route)
conflicts = gp_Astar.are_there_conflicts(route)
length = gp_Astar.route_length(route)

print(f"Conflicts: {conflicts}")
print(f"Length: {length}")
print(f"Time: {e_time}")
print(f"Explored nodes: {explored_nodes}")
print()