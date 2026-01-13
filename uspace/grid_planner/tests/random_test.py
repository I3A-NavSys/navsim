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

from uspace.grid_planner.grid_planner import GridPlanner


gp = GridPlanner()

random.seed(1)
routes_amount = 10
start_grid = -1000
end_grid = 1000
grid_nodes = 10
init_pos = []
end_pos = []
init_times = []
end_times = []

for r in range(routes_amount):
    i = random.randint(start_grid // gp.cell_side, end_grid // gp.cell_side)
    j = random.randint(start_grid // gp.cell_side, end_grid // gp.cell_side)

    i2 = random.randint(start_grid // gp.cell_side, end_grid // gp.cell_side)
    j2 = random.randint(start_grid // gp.cell_side, end_grid // gp.cell_side)

    while i == i2 and j == j2:
        i2 = random.randint(start_grid // gp.cell_side, end_grid // gp.cell_side)
        j2 = random.randint(start_grid // gp.cell_side, end_grid // gp.cell_side)

    init_pos.append((i, j, 60))
    end_pos.append((i2, j2, 60))

    init_times.append(r)
    end_times.append(r + 10)

# Define grid parameters
x = np.arange(start_grid-150, end_grid+201, 100)
y = np.arange(start_grid-200, end_grid+201, 100)

# ###########################################################################################
# A* COST PLOT
# Create figure and axis
def option_0_plot():
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(grid_nodes, grid_nodes))
    ax.set_title("Smallest routes")

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

def option_1_plot():
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(grid_nodes, grid_nodes))
    ax.set_title("Reaching end first routes")

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

def option_2_plot():
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(grid_nodes, grid_nodes))
    ax.set_title("Cheapest routes")

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
    
# ###########################################################################################
# A* COST ROUTES
def only_cost(axes, verbose=False, plot_routes=True):
    for i, ax in enumerate(axes):
        for r in range(routes_amount):
            if verbose:
                print("###################################")
                print(f"COMPUTING ROUTE {r}")
                print(f"Origin: {init_pos[r]} - {init_times[r]}")
                print(f"Destination: {end_pos[r]} - {end_times[r]}")
                print()

            route = gp.get_best_route(i, init_pos[r], end_pos[r], init_times[r], end_times[r], reverse=False)
            
            if route:
                conflicts = gp.are_there_conflicts(route)
                length = len(route)

                if verbose:
                    gp.print_route(route)
                    print(f"Conflicts: {conflicts}")
                    print(f"Length: {length}")

                if not conflicts:
                    gp.reserve_nodes(route)

                if plot_routes:
                    X = []
                    Y = []

                    for (i, j, l, s) in route:
                        X.append(i * 100 + 50 if l == "X" else i * 100)
                        Y.append(j * 100 + 50 if l == "Y" else j * 100)

                    line = ax.plot(X, Y, zorder=3, label=f"Route {r}: {length}")
                    sc = ax.scatter(X[0], Y[0], color="springgreen")
                    ax.scatter(X[-1], Y[-1], color="lightcoral")
                    ax.legend(loc="upper right", fontsize=8, bbox_to_anchor=(1.25, 1.15), borderaxespad=0.)

            else:
                if verbose:
                    print(f"Route could not be found in time slot {r}")

            if verbose:
                print("###################################")
                print()

        gp.clear_grid()

if __name__ == "__main__":
    ax_0 = option_0_plot()
    # ax_1 = option_1_plot()
    # ax_2 = option_2_plot()
    
    axes = []
    axes.append(ax_0)
    # axes.append(ax_1)
    # axes.append(ax_2)

    plt.grid(True)

    only_cost(axes, verbose=True, plot_routes=True)

    plt.show()