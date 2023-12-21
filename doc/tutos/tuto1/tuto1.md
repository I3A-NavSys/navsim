# Running your first simulation with Matlab connection

First, we open a scenario in the Gazebo simulator:

```bash
cd
cd code/navsim/ws/src/navsim_pkg/worlds
gazebo DroneChallenge.world
```

An environment should open with a gaming area of 10x10 meters. On a white base (the _vertiport_), there is a quadcopter (called _abejorro1_), and in the air, you could see three frames colored red, green, and blue, respectively.

![DroneChallenge](./img/DroneChallenge.png 'Drone Challenge scenario. :size=600px')

In a new terminal, we verify that ROS2 is running correctly:

```bash
$ros2 node list
/World
/abejorro1
/follow_cam
/onboard_cam
$
```

El comando muestra cuatro nodos activos en este momento:

- El nodo **/World** es la 