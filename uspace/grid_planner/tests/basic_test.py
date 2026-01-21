import sys
import os
import random
import matplotlib.pyplot as plt
import numpy as np
import time

#borrar consola
# os.system('cls')

# Añadir el directorio principal al sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))


from uspace.grid_planner.grid_planner import GridPlanner

random.seed(1)
gp = GridPlanner(max_route_length=1000)
start_grid = -500
end_grid = 500
exploration = []
exec_times = []

def build_plot():
    grid_nodes = 10
    x = np.arange(start_grid-150, end_grid+201, 100)
    y = np.arange(start_grid-200, end_grid+201, 100)

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

def draw_route(axe, origin, destination, start_time, end_time, verbose=False, reverse=False):
    if reverse:
        type_route = "REVERSE"
    else:
        type_route = "FORWARD"

    if verbose:
        og_node = gp.build_node_from_coords(origin, start_time, 0, None, 0)
        dest_node = gp.build_node_from_coords(destination, end_time, 0, None, 0)
        print("###################################")
        print(f"COMPUTING ROUTE {type_route}")
        print(f"Origin: {origin} - {start_time} -> ({og_node.i}, {og_node.j}, {og_node.l})")
        print(f"Destination: {destination} - {end_time} -> ({dest_node.i}, {dest_node.j}, {dest_node.l})")
        print()

    start_exec = time.time()
    route, explored_nodes = gp.get_route(origin, destination, start_time, end_time, reverse)
    end_exec = time.time()
    exploration.append(explored_nodes)
    exec_times.append(round(end_exec - start_exec, 7))

    if route:
        length = len(route)

        if verbose:
            # gp.print_route(route)
            print(f"Length: {length}")

        gp.reserve_nodes(route)

        # Plot the route
        X = []
        Y = []

        for (i, j, l, s) in route:
            X.append(i * 100 + 50 if l == "X" else i * 100)
            Y.append(j * 100 + 50 if l == "Y" else j * 100)

        line = axe.plot(X, Y, zorder=3, label=f"Route {type_route}: {length}")
        sc = axe.scatter(X[0], Y[0], color="springgreen")
        axe.scatter(X[-1], Y[-1], color="lightcoral")
        axe.legend(loc="upper right", fontsize=8, bbox_to_anchor=(1.25, 1.15), borderaxespad=0.)

    else:
        if verbose:
            print(f"Route could not be found.")

    if verbose:
        print("###################################")
        print()

    gp.clear_grid()

if __name__ == "__main__":
    axe = build_plot()
    routes_amount = 10
    origins = []
    destinations = []
    start_times = np.arange(0, routes_amount * 10, 10).tolist()
    # start_times = 0
    end_time = 0
    debug=False
    verbose=False

    for _ in range(routes_amount):
        x = random.randint(start_grid, end_grid)
        y = random.randint(start_grid, end_grid)
        origins.append([x, y, 0])

        x = random.randint(start_grid, end_grid)
        y = random.randint(start_grid, end_grid)
        destinations.append([x, y, 0])

    # origins = [[423, -175, 0]]
    # destinations = [[-469, -478, 0]]
    # start_times = [90]

    if debug:
        gp.debug = True
        gp.debug_figure = axe

    begin_exec_time = time.time()
    for i in range(routes_amount):
        if verbose:
            print(f"{i+1}º:")
        draw_route(axe, origins[i], destinations[i], start_times[i], end_time, verbose=verbose, reverse=False)
        # draw_route(axe, destinations[i], origins[i], start_times[i], end_time, verbose=verbose, reverse=True)
    finish_exec_time = time.time()

    print(f"Explored nodes per route: {exploration}")
    print(f"Execution times per route: {exec_times}")
    print(f"Total routes computed: {len(exploration)}")
    print(f"Average route explored nodes: {np.mean(exploration)} nodes")
    print(f"Average route execution time: {round(np.mean(exec_times), 7)} seconds")
    print(f"Total execution time: {round(finish_exec_time - begin_exec_time, 7)} seconds")
    
    plt.grid(True)
    plt.show()    