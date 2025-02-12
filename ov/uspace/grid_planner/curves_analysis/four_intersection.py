import sys
import os

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

import matplotlib.pyplot as plt
from matplotlib.text import Annotation
import numpy as np
import mplcursors
from uspace.flight_plan.flight_plan import FlightPlan

def build_plot():
    # Build the plot
    posFig = plt.figure("FOUR_INTERSECTION")
    xyzPosPlot = posFig.add_subplot(projection="3d")

    # Indicate axes' name
    xyzPosPlot.set_xlabel("x [m]")
    xyzPosPlot.set_ylabel("y [m]")
    xyzPosPlot.set_zlabel("z [m]")

    # Set title
    xyzPosPlot.set_title("Position 3D")

    # Set grid to True
    xyzPosPlot.grid(True)

def add_trace(fp):
    # Figure settings
    color = [0, 0.7, 1]

    # Get the trace
    tr = fp.trace(0.01)
    tr_x = tr[:, 1]
    tr_y = tr[:, 2]
    tr_z = tr[:, 3]

    # POSITION 3D
    # Get plot
    posFig = plt.figure("FOUR_INTERSECTION")
    subplots = posFig.get_axes()
    xyzPosPlot = subplots[0]

    # Set plot info
    xyzPosPlot.plot(tr_x, tr_y, tr_z, linewidth=2)

    # Get waypoints positions to highlight
    xPos = []
    yPos = []
    zPos = []

    for wp in fp.waypoints:
        xPos.append(wp.pos[0])
        yPos.append(wp.pos[1])
        zPos.append(wp.pos[2])

        if wp.pos[2] == 60:
            dy = 0

            if (wp.pos[1] // 100) % 2 == 0: dx = 100
            else:                           dx = -100

        else:
            dx = 0

            if (wp.pos[0] // 100) % 2 == 0: dy = 100
            else:                           dy = -100

        # Highlight waypoints positions
        xyzPosPlot.scatter(wp.pos[0], wp.pos[1], wp.pos[2], marker="o", color="blue", s=25)
        # xyzPosPlot.arrow(wp.pos[0], wp.pos[1], dx, dy, head_width=10, head_length=15, linewidth=0.5, linestyle=(0, (5, 10)),
        #                         length_includes_head=True, zorder=2)

    # Update limits to maintain scale in all axes
    xLim = max(np.abs(xyzPosPlot.get_xlim3d()))
    yLim = max(np.abs(xyzPosPlot.get_ylim3d()))
    zLim = max(np.abs(xyzPosPlot.get_zlim3d()))
    maxLim = max(xLim, yLim, zLim)

    xyzPosPlot.set_xlim3d(-maxLim, maxLim)
    xyzPosPlot.set_ylim3d(-maxLim, maxLim)
    xyzPosPlot.set_zlim3d(-maxLim, maxLim)

if __name__ == "__main__":
    # Flightplan 1
    fp1 = FlightPlan()
    fp1.set_waypoint(label="wp0", time=0, pos=[0, 0, 60], vel=[10, 0, 0])
    fp1.set_waypoint(label="wp1", time=10, pos=[100, 0, 60], vel=[10, 0, 0])
    # fp1.set_waypoint(label="wp2", time=20, pos=[200, 0, 60], vel=[10, 0, 0])
    # fp1.set_waypoint(label="wp3", time=30, pos=[250, 50, 100], vel=[0, 10, 0])
    fp1.set_waypoint(label="wp4", time=40, pos=[250, 150, 100], vel=[0, 10, 0])
    fp1.set_waypoint(label="wp5", time=50, pos=[250, 250, 100], vel=[0, 10, 0])

    # Flightplan 2
    fp2 = FlightPlan()
    fp2.set_waypoint(label="wp0", time=0, pos=[400, 100, 60], vel=[-10, 0, 0])
    fp2.set_waypoint(label="wp1", time=10, pos=[300, 100, 60], vel=[-10, 0, 0])
    # fp2.set_waypoint(label="wp2", time=20, pos=[200, 100, 60], vel=[-10, 0, 0])
    # fp2.set_waypoint(label="wp3", time=30, pos=[150, 50, 100], vel=[0, -10, 0])
    fp2.set_waypoint(label="wp4", time=40, pos=[150, -50, 100], vel=[0, -10, 0])
    fp2.set_waypoint(label="wp5", time=50, pos=[150, -150, 100], vel=[0, -10, 0])

    # Flightplan 3
    fp3 = FlightPlan()
    fp3.set_waypoint(label="wp0", time=0, pos=[150, 250, 100], vel=[0, -10, 0])
    fp3.set_waypoint(label="wp1", time=10, pos=[150, 150, 100], vel=[0, -10, 0])
    # fp3.set_waypoint(label="wp2", time=20, pos=[150, 50, 100], vel=[0, -10, 0])
    # fp3.set_waypoint(label="wp3", time=30, pos=[200, 0, 60], vel=[10, 0, 0])
    fp3.set_waypoint(label="wp4", time=40, pos=[300, 0, 60], vel=[10, 0, 0])
    fp3.set_waypoint(label="wp5", time=50, pos=[400, 0, 60], vel=[10, 0, 0])

    # Flightplan 4
    fp4 = FlightPlan()
    fp4.set_waypoint(label="wp0", time=0, pos=[250, -150, 100], vel=[0, 10, 0])
    fp4.set_waypoint(label="wp1", time=10, pos=[250, -50, 100], vel=[0, 10, 0])
    # fp4.set_waypoint(label="wp2", time=20, pos=[250, 50, 100], vel=[0, 10, 0])
    # fp4.set_waypoint(label="wp3", time=30, pos=[200, 100, 60], vel=[-10, 0, 0])
    fp4.set_waypoint(label="wp4", time=40, pos=[100, 100, 60], vel=[-10, 0, 0])
    fp4.set_waypoint(label="wp5", time=50, pos=[0, 100, 60], vel=[-10, 0, 0])

    fp1.connect_waypoints()
    fp2.connect_waypoints()
    fp3.connect_waypoints()
    fp4.connect_waypoints()

    # Plot
    build_plot()
    add_trace(fp1)
    add_trace(fp2)
    add_trace(fp3)
    add_trace(fp4)

    plt.show()