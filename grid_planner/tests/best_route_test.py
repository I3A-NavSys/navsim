import sys
import os
import random
import matplotlib.pyplot as plt
import numpy as np

#borrar consola
os.system('cls')

# Añadir el directorio principal al sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

##############################################################################

from grid_planner.grid_planner import GridPlanner


gp = GridPlanner()

random.seed(1)
start_grid = -500
end_grid = 500
grid_nodes = 10

# Define grid parameters
x = np.arange(start_grid-150, end_grid+201, 100)
y = np.arange(start_grid-200, end_grid+201, 100)

def build_plot():
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(grid_nodes, grid_nodes))
    ax.set_title("Best route")

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
ax = build_plot()

plt.grid(True)

def set_route_1(ax):
    takeoff_nodes = gp.get_take_off_nodes((-600, 0), 0)
    landing_nodes = gp.get_landing_nodes((600, 0))

    route, e_time, explored_nodes = gp.get_route(takeoff_nodes[0], landing_nodes[1])

    print("###################################")
    print(f"COMPUTING R1")
    print()
    gp.reserve_nodes(route)
    gp.print_route(route)

    X = []
    Y = []

    for node in route:
        X.append(node.i * 100 + 50 if node.L == "X" else node.i * 100)
        Y.append(node.j * 100 + 50 if node.L == "Y" else node.j * 100)

    line = ax.plot(X, Y, zorder=3, color="lightsteelblue",
                label=f"Reserved route {(takeoff_nodes[0].i, takeoff_nodes[0].j, takeoff_nodes[0].L, takeoff_nodes[0].s)}")
    sc = ax.scatter(X[0], Y[0], color="lightsteelblue")
    ax.scatter(X[-1], Y[-1], color="lightsteelblue")

def set_route_2(ax):
    takeoff_nodes = gp.get_take_off_nodes((-400, 0), 30)
    landing_nodes = gp.get_landing_nodes((-500, 100))

    route, e_time, explored_nodes = gp.get_route(takeoff_nodes[0], landing_nodes[1])

    print("###################################")
    print(f"COMPUTING R2")
    print()
    gp.reserve_nodes(route)
    gp.print_route(route)

    X = []
    Y = []

    for node in route:
        X.append(node.i * 100 + 50 if node.L == "X" else node.i * 100)
        Y.append(node.j * 100 + 50 if node.L == "Y" else node.j * 100)

    line = ax.plot(X, Y, zorder=3, color="lightsteelblue",
                label=f"Reserved route {(takeoff_nodes[0].i, takeoff_nodes[0].j, takeoff_nodes[0].L, takeoff_nodes[0].s)}")
    sc = ax.scatter(X[0], Y[0], color="lightsteelblue")
    ax.scatter(X[-1], Y[-1], color="lightsteelblue")

def get_best_route(ax, option, init_pos, end_pos, init_time, end_time):
    print("###################################")
    print(f"COMPUTING BEST ROUTE")
    print()

    best_route, routes = gp.get_best_route(option=option, init_pos=init_pos, end_pos=end_pos, init_time=init_time, end_time=end_time)
    gp.print_route(best_route)

    # if route is not None:
    #     X = []
    #     Y = []

    #     for node in route:
    #         X.append(node.i * 100 + 50 if node.L == "X" else node.i * 100)
    #         Y.append(node.j * 100 + 50 if node.L == "Y" else node.j * 100)

    #     line = ax.plot(X, Y, zorder=3, label="Best route")
    #     sc = ax.scatter(X[0], Y[0])
    #     ax.scatter(X[-1], Y[-1], color=sc.get_facecolor()[0])
    #     ax.legend(loc="upper right", fontsize=8, bbox_to_anchor=(1.25, 1.15), borderaxespad=0.)

    for route in routes:
        X = []
        Y = []

        for node in route:
            X.append(node.i * 100 + 50 if node.L == "X" else node.i * 100)
            Y.append(node.j * 100 + 50 if node.L == "Y" else node.j * 100)

        line = ax.plot(X, Y, zorder=3, label=f"Route {(route[0].s, route[-1].s, len(route))}, {(route[0].i, route[0].j)} - {(route[-1].i, route[-1].j)}")
        sc = ax.scatter(X[0], Y[0])
        ax.scatter(X[-1], Y[-1], color=sc.get_facecolor()[0])
        ax.legend(loc="upper right", fontsize=8, bbox_to_anchor=(1.25, 1.15), borderaxespad=0.)

    X = []
    Y = []

    for node in best_route:
        X.append(node.i * 100 + 50 if node.L == "X" else node.i * 100)
        Y.append(node.j * 100 + 50 if node.L == "Y" else node.j * 100)

    line = ax.plot(X, Y, zorder=3, label=f"Best route {(best_route[0].s, best_route[-1].s, len(best_route))}")
    sc = ax.scatter(X[0], Y[0])
    ax.scatter(X[-1], Y[-1], color=sc.get_facecolor()[0])
    ax.legend(loc="upper right", fontsize=8, bbox_to_anchor=(1.25, 1.15), borderaxespad=0.)

set_route_1(ax)
set_route_2(ax)
get_best_route(ax, option=0, init_pos=(-500, 0), end_pos=(-200, 0), init_time=-10, end_time=10)

plt.show()