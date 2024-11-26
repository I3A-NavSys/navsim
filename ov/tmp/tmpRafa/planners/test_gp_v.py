import sys
import os

#borrar consola
os.system('cls')

# Añadir el directorio principal al sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

##############################################################################


from grid_planner_v import GridPlanner


gp = GridPlanner()

TOpos = (320, 270)  # posición de despegue   (m)
TOtime = 33         # tiempo de despegue     (s)
Lpos  = (380, 735)  # posición de aterrizaje (m)

TOnodes = gp.GetTakeOffNodes(TOpos,TOtime)
Lnodes  = gp.GetLandingNodes(Lpos)

# route1 = gp.GetRoute(TOnodes[0], Lnodes[0])
# print(route1)
# print(gp.RouteLength(route1))
# print(gp.AreThereConflicts(route1))
# print()

# route2 = gp.GetRoute(TOnodes[0], Lnodes[1])
# print(route2)
# print(gp.RouteLength(route2))
# print(gp.AreThereConflicts(route2))
# print()

route3 = gp.GetRoute(TOnodes[1], Lnodes[0])
gp.print_route(route3)
print(gp.RouteLength(route3))
print(gp.AreThereConflicts(route3))
print()

# route4 = gp.GetRoute(TOnodes[1], Lnodes[1])
# print(route4)
# print(gp.RouteLength(route4))
# print(gp.AreThereConflicts(route4))
# print()