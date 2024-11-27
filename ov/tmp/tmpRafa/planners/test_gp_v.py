import sys
import os

#borrar consola
os.system('cls')

# Añadir el directorio principal al sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

##############################################################################


from grid_planner_v import GridPlanner as gp_v
from GridPlanner import GridPlanner as gp_r
from grid_planner_node import GridPlannerNode


gp_victor = gp_v()
gp_rafa = gp_r()

TOpos = (320, 270)  # posición de despegue   (m)
TOtime = 33         # tiempo de despegue     (s)
Lpos  = (380, 785)  # posición de aterrizaje (m)

TOnodes_rafa = gp_rafa.GetTakeOffNodes(TOpos,TOtime)
Lnodes_rafa  = gp_rafa.GetLandingNodes(Lpos)

TOnodes_victor = gp_victor.get_take_off_nodes(TOpos,TOtime)
Lnodes_victor  = gp_victor.get_landing_nodes(Lpos)

# Route 1
# print("-- Route 1 -----")
# print("No A*")
# route1 = gp_rafa.GetRoute(TOnodes_rafa[0], Lnodes_rafa[0])
# gp_rafa.print_route(route1)
# print(gp_rafa.RouteLength(route1))
# print(gp_rafa.AreThereConflicts(route1))
# print()

# print("A*")
# route1 = gp_victor.get_route(TOnodes_victor[0], Lnodes_victor[0])
# gp_victor.print_route(route1)
# print(gp_victor.route_length(route1))
# print(gp_victor.are_there_conflicts(route1))
# print()

# # Route 2
# print("-- Route 2 -----")
# print("No A*")
# route2 = gp_rafa.GetRoute(TOnodes_rafa[0], Lnodes_rafa[1])
# gp_rafa.print_route(route2)
# print(gp_rafa.RouteLength(route2))
# print(gp_rafa.AreThereConflicts(route2))
# print()

# print("A*")
# route2 = gp_victor.get_route(TOnodes_victor[0], Lnodes_victor[1])
# gp_victor.print_route(route2)
# print(gp_victor.route_length(route2))
# print(gp_victor.are_there_conflicts(route2))
# print()

# # Route 3
# print("-- Route 3 -----")
# print("No A*")
# route3 = gp_rafa.GetRoute(TOnodes_rafa[1], Lnodes_rafa[0])
# gp_rafa.print_route(route3)
# print(gp_rafa.RouteLength(route3))
# print(gp_rafa.AreThereConflicts(route3))
# print()

# print("A*")
# route3 = gp_victor.get_route(TOnodes_victor[1], Lnodes_victor[0])
# gp_victor.print_route(route3)
# print(gp_victor.route_length(route3))
# print(gp_victor.are_there_conflicts(route3))
# print()

# # Route 4
# print("-- Route 4 -----")
# print("No A*")
# route4 = gp_rafa.GetRoute(TOnodes_rafa[1], Lnodes_rafa[1])
# gp_rafa.print_route(route4)
# print(gp_rafa.RouteLength(route4))
# print(gp_rafa.AreThereConflicts(route4))
# print()

# print("A*")
# route4 = gp_victor.get_route(TOnodes_victor[1], Lnodes_victor[1])
# gp_victor.print_route(route4)
# print(gp_victor.route_length(route4))
# print(gp_victor.are_there_conflicts(route4))
# print()

print("-------------------")
route = gp_victor.get_route(GridPlannerNode(2, 4, 'X', 5, 0, None), GridPlannerNode(8, 4, 'X', -1, 0, None))

gp_victor.print_route(route)
print(gp_victor.route_length(route))
conflicts = gp_victor.are_there_conflicts(route)
print(conflicts)
print()
if not conflicts:
    gp_victor.reserve_nodes(route)

print("-------------------")
route = gp_victor.get_route(GridPlannerNode(1, 4, 'X', 4, 0, None), GridPlannerNode(8, 4, 'X', -1, 0, None))

gp_victor.print_route(route)
print(gp_victor.route_length(route))
conflicts = gp_victor.are_there_conflicts(route)
print(conflicts)
print()
if not conflicts:
    gp_victor.reserve_nodes(route)

print("-------------------")
route = gp_victor.get_route(GridPlannerNode(1, 8, 'X', 4, 0, None), GridPlannerNode(8, 4, 'X', -1, 0, None))

gp_victor.print_route(route)
print(gp_victor.route_length(route))
conflicts = gp_victor.are_there_conflicts(route)
print(conflicts)
print()
if not conflicts:
    gp_victor.reserve_nodes(route)