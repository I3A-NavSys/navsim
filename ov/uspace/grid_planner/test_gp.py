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


from grid_planner import GridPlanner as gp_v, GridNode
from tmp.tmpRafa.planners.GridPlanner import GridPlanner as gp_r


gp_victor = gp_v()
gp_rafa = gp_r()

TOpos = (-3000, -3000)  # posición de despegue   (m)
TOtime = 33         # tiempo de despegue     (s)
Lpos  = (3000, 3000)  # posición de aterrizaje (m)

# TOnodes_rafa = gp_rafa.GetTakeOffNodes(TOpos,TOtime)
# Lnodes_rafa  = gp_rafa.GetLandingNodes(Lpos)

# TOnodes_victor = gp_victor.get_take_off_nodes(TOpos,TOtime)
# Lnodes_victor  = gp_victor.get_landing_nodes(Lpos)

# Route 1
# print("-- Route 1 -----")
# print("No A*")
# route1 = gp_rafa.GetRoute(TOnodes_rafa[0], Lnodes_rafa[0])
# gp_rafa.print_route(route1)
# print(gp_rafa.RouteLength(route1))
# print(gp_rafa.AreThereConflicts(route1))
# print()

# print("A*")
# route1, elapsed_time, explored_nodes = gp_victor.get_route(TOnodes_victor[0], Lnodes_victor[0])
# gp_victor.print_route(route1)
# print(gp_victor.route_length(route1))
# print(gp_victor.are_there_conflicts(route1))
# print()

# Route 2
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

# print("###################################")
# print(f"COMPUTING ROUTE")
# print(f"Origin: 2, 4, X, 5")
# print(f"Destination: 8, 4, X, -1")
# print()

# route, e_time, explored_nodes = gp_victor.get_route(GridNode(2, 4, 'X', 5, 0, None), GridNode(8, 4, 'X', -1, 0, None))
# gp_victor.print_route(route)

# conflicts = gp_victor.are_there_conflicts(route)
# if not conflicts:
#     gp_victor.reserve_nodes(route)
    
# print(f"Conflicts: {conflicts}")
# print(f"Length: {gp_victor.route_length(route)}")
# print(f"Time: {e_time}")
# print(f"Explored nodes: {explored_nodes}")
# print("--------------")
# print("###################################")
# print()

# print("###################################")
# print(f"COMPUTING ROUTE")
# print(f"Origin: 2, 4, X, 5")
# print(f"Destination: 8, 4, X, -1")
# print()

# route, e_time, explored_nodes = gp_victor.get_route(GridNode(1, 4, 'X', 4, 0, None), GridNode(8, 4, 'X', -1, 0, None))
# gp_victor.print_route(route)

# conflicts = gp_victor.are_there_conflicts(route)
# if not conflicts:
#     gp_victor.reserve_nodes(route)
    
# print(f"Conflicts: {conflicts}")
# print(f"Length: {gp_victor.route_length(route)}")
# print(f"Time: {e_time}")
# print(f"Explored nodes: {explored_nodes}")
# print("--------------")
# print("###################################")
# print()

# route, e_time, explored_nodes = gp_victor.get_route(GridNode(1, 8, 'X', 4, 0, None), GridNode(8, 4, 'X', -1, 0, None))
# gp_victor.print_route(route)

# conflicts = gp_victor.are_there_conflicts(route)
# if not conflicts:
#     gp_victor.reserve_nodes(route)
    
# print(f"Conflicts: {conflicts}")
# print(f"Length: {gp_victor.route_length(route)}")
# print(f"Time: {e_time}")
# print(f"Explored nodes: {explored_nodes}")
# print("--------------")
# print("###################################")
# print()

random.seed(1)
routes_amount = 5

# Define grid parameters
x = np.arange(50, 1001, 100)
y = np.arange(0, 1001, 100)

# Create figure and axis
fig, ax = plt.subplots(figsize=(10, 10))

ax.set_xticks(np.arange(0, 1000, 100))  # Ensure grid spacing is 100 units
ax.set_yticks(np.arange(0, 1000, 100))
ax.grid(which="both", color="lightgrey", linestyle="-", linewidth=0.3, zorder=1)

# Plot points
for xi in x:
    for yi in y:
        ax.scatter(xi, yi, color="red", s=10, zorder=4)
        ax.scatter(yi, xi, color="orange", s=10, zorder=4)

# Draw horizontal and vertical lines with arrows
for xi in x:
    for yi in y:
        # Horizontal lines
        if xi < 1100:  # Avoid drawing beyond the grid
            dx = 100 if yi % 200 == 0 else -100  # Even Y goes right, odd Y goes left
            ax.arrow(xi, yi, dx, 0, head_width=7, head_length=12, linewidth=0.5,
                    fc='red', ec='red', linestyle="dotted", length_includes_head=True, zorder=2)
        
        # Vertical lines
        if yi < 1100:  # Avoid drawing beyond the grid
            dy = 100 if (xi // 100) % 2 == 0 else -100  # Even X goes up, odd X goes down
            ax.arrow(xi-50, yi-50, 0, dy, head_width=7, head_length=12, linewidth=0.5,
                    fc='gold', ec='gold', linestyle="dotted", length_includes_head=True, zorder=2)

# Set axis limits and labels
ax.set_xlim(-5, 1005)
ax.set_ylim(-5, 1005)
ax.set_aspect('equal')
ax.set_xlabel('Position X (m)')
ax.set_ylabel('Position Y (m)')

ax.legend(loc="upper right", fontsize=12)

plt.grid(True)

for c in range(routes_amount):
    t_takeoff = c

    i = random.randint(0, 1000)
    j = random.randint(0, 1000)

    i2 = random.randint(0, 1000)
    j2 = random.randint(0, 1000)

    while i == i2 and j == j2:
        i2 = random.randint(0, 1000)
        j2 = random.randint(0, 1000)

    takeoff_nodes = gp_victor.get_take_off_nodes((i, j), t_takeoff)
    landing_nodes = gp_victor.get_landing_nodes((i2, j2))

    print("###################################")
    print(f"COMPUTING ROUTE")
    print(f"Origin: {takeoff_nodes[0].i, takeoff_nodes[0].j, takeoff_nodes[0].L, takeoff_nodes[0].s}")
    print(f"Destination: {landing_nodes[0].i, landing_nodes[0].j, landing_nodes[0].L, landing_nodes[0].s}")
    print()

    route, e_time, explored_nodes = gp_victor.get_route(takeoff_nodes[0], landing_nodes[0])
    gp_victor.print_route(route)
    conflicts = gp_victor.are_there_conflicts(route)
    print(f"Conflicts: {conflicts}")
    print(f"Length: {gp_victor.route_length(route)}")
    print(f"Time: {e_time}")
    print(f"Explored nodes: {explored_nodes}")
    print("--------------")
    print("###################################")
    print()

    if not conflicts:
        gp_victor.reserve_nodes(route)

    X = []
    Y = []

    for node in route:
        X.append(node.i * 100)
        Y.append(node.j * 100)

    line = ax.plot(X, Y, zorder=3, label=f"Route {c}")
    sc = ax.scatter(X[0], Y[0])
    ax.scatter(X[-1], Y[-1], color=sc.get_facecolor()[0])

    # handles, labels = ax.get_legend_handles_labels()
    # handles.append(line)
    # labels.append("Route" + str(i))
    # ax.legend(handles=handles, labels=labels, loc="upper right", fontsize=12)
    ax.legend()

plt.show()