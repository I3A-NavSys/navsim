# 01: Running your first simulation

In this tutorial you will learn to execute a simulation by opening a scenario, placing some UAVs over the terrain and 
controlling them by means of a Thrustmaster joystick (other joysticks may work but it is no guaranteed).

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

## Control the UAV

Now we are going to control the UAV using the joystick. For this we need to work with our own extension named 
*Navsim - Manual Controller*.
In this extension we have two main parts, the *Control* and *Visualization* tabs.
- Control: 
    - Here we have configurable parameters 

