import sys
import os
import random
import matplotlib.pyplot as plt
import numpy as np

#borrar consola
os.system('cls')

# Añadir el directorio principal al sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

##############################################################################

from grid_planner import GridPlanner


gp = GridPlanner()

random.seed(1)
routes_amount = 200
alternative_routes_amount = 3
respect_limits = True
start_grid = 0
end_grid = 1000
grid_nodes = 10
takeoff_nodes_list = []
landing_nodes_list = []
routes = []

for k in range(routes_amount):
    t_takeoff = k * 10

    i = random.randint(start_grid, end_grid)
    j = random.randint(start_grid, end_grid)

    i2 = random.randint(start_grid, end_grid)
    j2 = random.randint(start_grid, end_grid)

    while i == i2 and j == j2:
        i2 = random.randint(start_grid, end_grid)
        j2 = random.randint(start_grid, end_grid)

    takeoff_nodes = gp.get_take_off_nodes((i, j), t_takeoff)
    landing_nodes = gp.get_landing_nodes((i2, j2))

    takeoff_nodes_list.append(takeoff_nodes)
    landing_nodes_list.append(landing_nodes)

# Define grid parameters
x = np.arange(start_grid-150, end_grid+201, 100)
y = np.arange(start_grid-200, end_grid+201, 100)

# ###########################################################################################
# A* HEURISTICS PLOT
def heuristics_plot():
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(grid_nodes, grid_nodes))
    ax.set_title("A* only heuristics")

    x_sticks = np.arange(start_grid-200, end_grid+200, 100)
    y_sticks = np.arange(start_grid-200, end_grid+200, 100)

    ax.set_xticks(x_sticks)  # Ensure grid spacing is 100 units
    ax.set_yticks(y_sticks)
    ax.set_xticklabels(x_sticks // 100)
    ax.set_yticklabels(y_sticks // 100)

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
            if xi < end_grid+300:  # Avoid drawing beyond the grid
                dx = 100 if yi % 200 == 0 else -100  # Even Y goes right, odd Y goes left
                ax.arrow(xi, yi, dx, 0, head_width=10, head_length=15, linewidth=0.5,
                        fc='red', ec='red', linestyle=(0, (5, 10)), length_includes_head=True, zorder=2)
            
            # Vertical lines
            if yi < end_grid+300:  # Avoid drawing beyond the grid
                dy = 100 if (xi // 100) % 2 == 0 else -100  # Even X goes up, odd X goes down
                ax.arrow(xi-50, yi-50, 0, dy, head_width=10, head_length=15, linewidth=0.5,
                        fc='gold', ec='gold', linestyle=(0, (5, 10)), length_includes_head=True, zorder=2)

    # Set axis limits and labels
    ax.set_xlim(start_grid-205, end_grid+205)
    ax.set_ylim(start_grid-205, end_grid+205)
    ax.set_aspect('equal')
    ax.set_xlabel('Position X (m)')
    ax.set_ylabel('Position Y (m)')

    return ax
ax = heuristics_plot()

# ###########################################################################################
# A* COST PLOT
def cost_plot():
    # Create figure and axis
    fig, ax2 = plt.subplots(figsize=(grid_nodes, grid_nodes))
    ax2.set_title("A* only cost")

    x_sticks = np.arange(start_grid-200, end_grid+200, 100)
    y_sticks = np.arange(start_grid-200, end_grid+200, 100)

    ax2.set_xticks(x_sticks)  # Ensure grid spacing is 100 units
    ax2.set_yticks(y_sticks)
    ax2.set_xticklabels(x_sticks // 100)
    ax2.set_yticklabels(y_sticks // 100)

    ax2.grid(which="both", color="lightgrey", linestyle="-", linewidth=0.3, zorder=1)

    # Plot points
    for xi in x:
        for yi in y:
            ax2.scatter(xi, yi, color="red", s=10, zorder=4)
            ax2.scatter(yi, xi, color="orange", s=10, zorder=4)

    # Draw horizontal and vertical lines with arrows
    for xi in x:
        for yi in y:
            # Horizontal lines
            if xi < end_grid+300:  # Avoid drawing beyond the grid
                dx = 100 if yi % 200 == 0 else -100  # Even Y goes right, odd Y goes left
                ax2.arrow(xi, yi, dx, 0, head_width=10, head_length=15, linewidth=0.5,
                        fc='red', ec='red', linestyle=(0, (5, 10)), length_includes_head=True, zorder=2)
            
            # Vertical lines
            if yi < end_grid+300:  # Avoid drawing beyond the grid
                dy = 100 if (xi // 100) % 2 == 0 else -100  # Even X goes up, odd X goes down
                ax2.arrow(xi-50, yi-50, 0, dy, head_width=10, head_length=15, linewidth=0.5,
                        fc='gold', ec='gold', linestyle=(0, (5, 10)), length_includes_head=True, zorder=2)

    # Set axis limits and labels
    ax2.set_xlim(start_grid-205, end_grid+205)
    ax2.set_ylim(start_grid-205, end_grid+205)
    ax2.set_aspect('equal')
    ax2.set_xlabel('Position X (m)')
    ax2.set_ylabel('Position Y (m)')

    return ax2
ax2 = cost_plot()

plt.grid(True)

# ###########################################################################################
# A* HEURISTICS ROUTES
def only_heuristics(verbose=True):
    for c in range(routes_amount):
        if verbose:
            print("###################################")
            print(f"COMPUTING ROUTE {c}")
            print(f"Origin: {takeoff_nodes_list[c][0].i, takeoff_nodes_list[c][0].j, takeoff_nodes_list[c][0].L, takeoff_nodes_list[c][0].s}")
            print(f"Destination: {landing_nodes_list[c][0].i, landing_nodes_list[c][0].j, landing_nodes_list[c][0].L, landing_nodes_list[c][0].s}")
            print()

        route, e_time, explored_nodes = gp.get_route(takeoff_nodes_list[c][0], landing_nodes_list[c][0], cost_only=False, 
                                                    respect_limits=respect_limits)
        
        if route is not None:
            routes.append(route)
            conflicts = gp.are_there_conflicts(route)
            length = gp.route_length(route)

            if verbose:
                gp.print_route(route)
                print(f"Conflicts: {conflicts}")
                print(f"Length: {length}")
                print(f"Time: {e_time}")
                print(f"Explored nodes: {explored_nodes}")

            if not conflicts:
                gp.reserve_nodes(route)

            X = []
            Y = []

            for node in route:
                X.append(node.i * 100 + 50 if node.L == "X" else node.i * 100)
                Y.append(node.j * 100 + 50 if node.L == "Y" else node.j * 100)

            line = ax.plot(X, Y, zorder=3, color="lightsteelblue")
            sc = ax.scatter(X[0], Y[0], color="lightsteelblue")
            ax.scatter(X[-1], Y[-1], color="lightsteelblue")
            # ax.legend(loc="upper right", fontsize=8, bbox_to_anchor=(1.25, 1.15), borderaxespad=0.)

            line = ax2.plot(X, Y, zorder=3, color="lightsteelblue")
            sc = ax2.scatter(X[0], Y[0], color="lightsteelblue")
            ax2.scatter(X[-1], Y[-1], color="lightsteelblue")

        else:
            if verbose:
                print(f"Route could not be found in time slot {takeoff_nodes[0].s}")
                print("--------------")
                print(f"Time: {e_time}")
                print(f"Explored nodes: {explored_nodes}")
                print("--------------")
        if verbose:
            print("###################################")
            print()
only_heuristics(verbose=False)
print(gp.grid)
# ###########################################################################################
# Alternative routes given a range of time
t_takeoff = routes_amount*10 // 2
step = 30
t_takeoff = t_takeoff - step * alternative_routes_amount - step

i = random.randint(start_grid, end_grid)
j = random.randint(start_grid, end_grid)

i2 = random.randint(start_grid, end_grid)
j2 = random.randint(start_grid, end_grid)

while i == i2 and j == j2:
    i2 = random.randint(start_grid, end_grid)
    j2 = random.randint(start_grid, end_grid)

def alt_routes(cost_only, t_takeoff, verbose=True):
    for alt_route in range(alternative_routes_amount*2):
        takeoff_nodes = gp.get_take_off_nodes((i, j), t_takeoff)
        landing_nodes = gp.get_landing_nodes((i2, j2))

        if verbose:
            print("###################################")
            print(f"COMPUTING ALTERNATIVE {alt_route}")
            print(f"Origin: {takeoff_nodes[0].i, takeoff_nodes[0].j, takeoff_nodes[0].L, takeoff_nodes[0].s}")
            print(f"Destination: {landing_nodes[0].i, landing_nodes[0].j, landing_nodes[0].L, landing_nodes[0].s}")
            print()

        route, e_time, explored_nodes = gp.get_route(takeoff_nodes[0], landing_nodes[0], 
                                                     cost_only=cost_only, respect_limits=respect_limits)
        
        if route is not None:
            conflicts = gp.are_there_conflicts(route)
            length = gp.route_length(route)

            if verbose:
                gp.print_route(route)
                print(f"Conflicts: {conflicts}")
                print(f"Length: {length}")
                print(f"Time: {e_time}")
                print(f"Explored nodes: {explored_nodes}")

            X = []
            Y = []

            for node in route:
                X.append(node.i * 100 + 50 if node.L == "X" else node.i * 100)
                Y.append(node.j * 100 + 50 if node.L == "Y" else node.j * 100)

            if not cost_only:
                line = ax.plot(X, Y, zorder=3, label=f"RouteA {alt_route}: {length} - {round(e_time, 3)}s - {explored_nodes}")
                sc = ax.scatter(X[0], Y[0])
                ax.scatter(X[-1], Y[-1], color=sc.get_facecolor()[0])
                ax.legend(loc="upper right", fontsize=8, bbox_to_anchor=(1.25, 1.15), borderaxespad=0.)
            
            else:
                line = ax2.plot(X, Y, zorder=3, label=f"Route {alt_route}: {length} - {round(e_time, 3)}s - {explored_nodes}")
                sc = ax2.scatter(X[0], Y[0])
                ax2.scatter(X[-1], Y[-1], color=sc.get_facecolor()[0])
                ax2.legend(loc="upper right", fontsize=8, bbox_to_anchor=(1.25, 1.15), borderaxespad=0.)
        
        else:
            if verbose:
                print(f"Route could not be found in time slot {takeoff_nodes[0].s}")
                print("--------------")
                print(f"Time: {e_time}")
                print(f"Explored nodes: {explored_nodes}")
                print("--------------")
        
        if verbose:
            print("###################################")
            print()

        t_takeoff += step
alt_routes(False, t_takeoff, verbose=True)
alt_routes(True, t_takeoff, verbose=False)

plt.show()