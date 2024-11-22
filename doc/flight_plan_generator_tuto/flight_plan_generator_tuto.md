# 03: Executing a Flight Plan

In this tutorial you will learn how to create a flight plan and assign it to a drone.

## Launch Isaac Sim

First, open the simulation environment NVIDIA Isaac Sim. Once opened, it will offer a similar aspect as the following
picture (the panels' location may differ depending on the personal configuration):

![Isaac Sim](./img/isaac_sim_launched.png)

## Launch the scenario

In the *Content* panel, find the file `navsim/ov/assets/worlds/campus/campus.usd` and doble click on it.
It will open a scenario where our campus is detailed.

![campus.usd](./img/campus_usd.png)

Under *Stage* panel, find `World/Vertiports/minidrone_vertiport` prim (a prim is how models are called in USD), select 
it and press *F* key to focus (make zoom) on it.
Then drag and drop the file `navsim/ov/fleet/UAM_minidrone/UAM_minidrone.usd` over the vertiport. It will appear a 
quadcopter in the scene. You can elevate it a bit and click on *PLAY* button to start the
simulation. You will see that the UAV falls. Besides, you can have a look at its internal components under the *Stage*
panel.

https://github.com/user-attachments/assets/68807db9-35ee-4cd5-9f48-3e62e3f2b44c

## Build a Flight Plan

Before beginning with the tutorial, you need to know that our flight plan is based on *waypoints*. These waypoints
contain information about the position and velocity the UAV must have at a specific time slot, so in order to build a
flight plan you must create a set of waypoints which will be ordered in time and then interconnected.  

For this task we have developed our own custom extension named *NavSim - Flight Plan Generator*. This extension allows
you to build a flight plan using UI, no need to use code.
It is divided into three different parts, the drone selector, to choose the UAV to send flight plan to, the waypoint
builder and the generated waypoint list.  

In regards to the waypoint builder, it has the following parameters:
- Time: time slot in which the waypoint will be inserted
- Position: the position of the waypoint in a 3D space
- Velocity: the linear velocity to reach next waypoint
- Fly over: wheter this waypoint is mandatory to pass trough or not
- WaypointID: an identifier of the waypoint

![waypoint_builder](./img/waypoint_builder.png)

Each parameter has a checkbox on its right, this is intended to tell the extension whether to have into account the
parameter or not. If not, the extension will set the corresponding values automatically.  

Finally there are two buttons, one to add the waypoint, and another to clean the waypoint list.  

Once you know how the extension works, let's give it a try. Locate the extension and create the waypoints below.  

Waypoint 1

![wp1](./img/wp1.png)

Waypoint 2

![wp2](./img/wp2.png)

Waypoint 3

![wp3](./img/wp3.png)

Waypoint 4

![wp4](./img/wp4.png)

Once all waypoints have been added, the resulting waypoint list should be as follows:

![resulting_waypoint_list](./img/resulting_waypoint_list.png)

Finally, click on *REFRESH* button at the top of the extension, select an UAV among the options, click on *PLAY*
button to start the simulation and click on *Send Flight Plan* at the bottom of the extension to submit the flight plan.

https://github.com/user-attachments/assets/38b11870-94a4-4309-9640-19287bd6b189

