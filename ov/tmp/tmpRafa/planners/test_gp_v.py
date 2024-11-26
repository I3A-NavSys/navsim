import sys
import os

#borrar consola
os.system('cls')

# Añadir el directorio principal al sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

##############################################################################


from grid_planner_v import GridPlanner as gp_v
from GridPlanner import GridPlanner as gp_r


gp_victor = gp_v()
gp_rafa = gp_r()

TOpos = (320, 270)  # posición de despegue   (m)
TOtime = 33         # tiempo de despegue     (s)
Lpos  = (380, 735)  # posición de aterrizaje (m)

TOnodes_rafa = gp_rafa.GetTakeOffNodes(TOpos,TOtime)
Lnodes_rafa  = gp_rafa.GetLandingNodes(Lpos)

TOnodes_victor = gp_victor.GetTakeOffNodes(TOpos,TOtime)
Lnodes_victor  = gp_victor.GetLandingNodes(Lpos)

# Route 1
print("-- Route 1 -----")
print("No A*")
route1 = gp_rafa.GetRoute(TOnodes_rafa[0], Lnodes_rafa[0])
gp_rafa.print_route(route1)
print(gp_rafa.RouteLength(route1))
print(gp_rafa.AreThereConflicts(route1))
print()

print("A*")
route1 = gp_victor.GetRoute(TOnodes_victor[0], Lnodes_victor[0])
gp_victor.print_route(route1)
print(gp_victor.RouteLength(route1))
print(gp_victor.AreThereConflicts(route1))
print()

# Route 2
print("-- Route 2 -----")
print("No A*")
route2 = gp_rafa.GetRoute(TOnodes_rafa[0], Lnodes_rafa[1])
gp_rafa.print_route(route2)
print(gp_rafa.RouteLength(route2))
print(gp_rafa.AreThereConflicts(route2))
print()

print("A*")
route2 = gp_victor.GetRoute(TOnodes_victor[0], Lnodes_victor[1])
gp_victor.print_route(route2)
print(gp_victor.RouteLength(route2))
print(gp_victor.AreThereConflicts(route2))
print()

# Route 3
print("-- Route 3 -----")
print("No A*")
route3 = gp_rafa.GetRoute(TOnodes_rafa[1], Lnodes_rafa[0])
gp_rafa.print_route(route3)
print(gp_rafa.RouteLength(route3))
print(gp_rafa.AreThereConflicts(route3))
print()

print("A*")
route3 = gp_victor.GetRoute(TOnodes_victor[1], Lnodes_victor[0])
gp_victor.print_route(route3)
print(gp_victor.RouteLength(route3))
print(gp_victor.AreThereConflicts(route3))
print()

# Route 4
print("-- Route 4 -----")
print("No A*")
route4 = gp_rafa.GetRoute(TOnodes_rafa[1], Lnodes_rafa[1])
gp_rafa.print_route(route4)
print(gp_rafa.RouteLength(route4))
print(gp_rafa.AreThereConflicts(route4))
print()

print("A*")
route4 = gp_victor.GetRoute(TOnodes_victor[1], Lnodes_victor[1])
gp_victor.print_route(route4)
print(gp_victor.RouteLength(route4))
print(gp_victor.AreThereConflicts(route4))
print()