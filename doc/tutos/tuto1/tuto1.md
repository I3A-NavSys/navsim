# 01: Running your first simulation with Matlab connection

In this tutorial, we are going to introduce a programming competition held at the School of Computer Engineering in Albacete (University of Castilla-La Mancha).
The challenge is aimed at promoting computational thinking among students in Compulsory Secondary Education.


## Launch the scenario

First, we open a scenario in the Gazebo simulator:

```bash
cd
cd code/navsim/ws/src/navsim_pkg/worlds
gazebo DroneChallenge.world
```
An environment should open with a gaming area of 10x10 meters. On a white base (the _vertiport_), there is a quadcopter (called _abejorro_). Floating in the air, you could see three frames colored red, green, and blue, respectively.

![DroneChallenge](./img/DroneChallenge.png)


In a new terminal, we can verify that ROS is running correctly:
```bash
ros2 node list
```
The command shows four active nodes at the moment.


## Check world node

The node **/World** is associated with the loaded scenario. 

```bash
ros2 topic list | grep NavSim
```
We observe that this node generates a topic **/NavsSim/Time** where we can check the simulation time.

```bash
ros2 topic type /NavSim/Time
ros2 interface show builtin_interfaces/msg/Time
ros2 topic echo /NavSim/Time
```
With these commands, we can determine the structure of the transmitted message, and observe as the data refreshes 10 times per second.

```bash
ros2 service list | grep NavSim
```
We check that there are three services associated with this node, named **/NavSim/DeployModel**, **/NavSim/RemoveModel**, and **/NavSim/SimControl**, respectively.

The **/NavSim/RemoveModel** service allows us to remove an element from the scenario.
```bash
ros2 service type /NavSim/RemoveModel 
ros2 interface show navsim_msgs/srv/RemoveModel
```
We see that we can call the service simply by indicating the name of the object we want to remove. As an example, let's remove the landing pad with the command:
```bash
ros2 service call /NavSim/RemoveModel navsim_msgs/srv/RemoveModel "{name: 'vertiport'}"
```
We see that the landing pad disappears, and the drone falls to the ground.
Before proceeding, close Gazebo and let's load the original scenario again to recover the vertiport.

```bash
gazebo DroneChallenge.world
```


## Check UAV node

Each UAV generates its own ROS node to interact with the environment. In this case, the node for the quadcopter is named **/abejorro**.
This node manages the transmission of telemetry information and the reception and execution of control commands.


### Telemetry

All existing UAVs will publish their telemetry information on the **/NavSim/_UAVidentifier_/Telemetry** topic.

```bash
ros2 topic list | grep Telemetry
ros2 topic info /NavSim/abejorro/Telemetry
```
We can see that there is a node transmitting messages on this topic. 

```bash
ros2 interface show navsim_msgs/msg/Telemetry
```
The telemetry message contains:
- UAV identifier (to allow the management a swarm with the same callback function)
- UAV position and orientation
- UAV world linear velocity
- UAV body axes angular velocities
- Data generation time

```bash
ros2 topic echo /NavSim/abejorro/Telemetry
```
The refresh rate of telemetry information varies according to the application's requirements, ranging from 10 Hz (this case) to 1 Hz.


### Navigation

Each UAV has its own **RemoteCommand** control topic:

```bash
ros2 topic list | grep RemoteCommand
ros2 topic type /NavSim/abejorro/RemoteCommand
ros2 interface show navsim_msgs/msg/RemoteCommand 
```
We observe that the control topic is named **/NavSim/abejorro/RemoteCommand**. The command includes:
- UAV identifier (only required in cases where the topic refers to a swarm)
- Activation/deactivation of rotors
- Commanded linear velocity (expressed in horizon axes for this kind of drone)
- Commanded angular velocity (only around the vertical axis)
- Command expiration time.

As an example of use, we can move the quapcopter publishing the following commands:
```bash
ros2 topic pub -1 /NavSim/abejorro/RemoteCommand navsim_msgs/msg/RemoteCommand "{'on': true, 'vel': {'linear': {z: 1}}, 'duration': {'sec': 1}}"
ros2 topic pub -1 /NavSim/abejorro/RemoteCommand navsim_msgs/msg/RemoteCommand "{'on': true, 'vel': {'linear': {x: 1}, 'angular': {z: 1}}, 'duration': {'sec': 6}}"
ros2 topic pub -1 /NavSim/abejorro/RemoteCommand navsim_msgs/msg/RemoteCommand "{'on': false}"
```
![Drone Flying](./img/drone_flying.png)


## Check UAV cameras

```bash
ros2 node list
```
This quadcopter has two cameras. Each of them generates its own node for image transmission:
- **/onboard_cam** is a real camera mounted on the front of the fuselage. 
- **/follow_cam** is a virtual camera located at a certain height behind the aircraft. 

```bash
ros2 topic list | grep _cam
ros2 topic echo /NavSim/abejorro/onboard_cam/image_raw --no-arr
ros2 topic echo /NavSim/abejorro/follow_cam/image_raw --no-arr
```
Each camera generates two topics, transmitting camera information and raw image respectively. 
- **/NavSim/abejorro/onboard_cam/image_raw** is transmitting 320x240 RGB pixels at 10 FPS. 
- **/NavSim/abejorro/follow_cam/image_raw** is transmitting 480x320 RGB pixels at 10 FPS.



## Launch the human pilot interface

> Warning: This step requires having a joystick connected to the computer running Matlab / Simulink. If you don't have one, you can skip this section and proceed to the next one.
> This tutorial has been conducted using a **Thrustmaster T.Flight Stick X** joystick model.

In the Matlab environment, navigate to the `navsim/matlab/operators/DroneChallenge` directory. From there, open the `HumanPilot.slx` model.
Press the **Run** button to execute the model. After compilation, two additional windows will open, displaying video captured by the drone cameras. 
You can rearrange, resize, or close these windows as desired.

![Onboard camera](./img/onboardCAM.png)

![Follow camera](./img/followCAM.png)


On the right, the model provides a speedometer and a maximum speed limiter. It is recommended to set the speed below 1 m/s in the initial attempts.
![Human pilot interface](./img/HumanPilot.png)

On the left, there is a joystick diagram including a circular LED. It indicates whether the quadcopter rotors are on (green) or off (red).
- To turn on the motors, press button 1 on the joystick (index finger). Initially, the drone stabilizes, and it may not be immediately apparent that the motors are on. Check the LED if in doubt.
- To turn off the motors, press button 2 on the joystick (thumb). The drone will descend to the ground if it was hovering in the air.

> If the joystick doesn't respond, double-click on the **Joystick drone 1 / Joystick input** block to access its configuration options. Modify the value of the **Joystick ID** parameter with different port numbers until you find the correct one.

With the motors on, the drone stabilizes automatically. Then, you can:
- ascend/descend: employing the thrust lever.
- do planar movement: moving the stick.
- rotate: rotating the stick.

Try piloting the drone to pass through the red, green, and blue frames in order and then return to the vertiport. Record the time from when the motors are activated until they are deactivated again. The pilot with the shortest completion time will be the winner.

Enjoy the game!
![Drone crossing the green frame](./img/drone_crossing.png)


## Launch the computer pilot

Stop the human pilot interface if it is still running. From Matlab, open the **ComputerPilot.slx** model.

![Computer pilot](./img/ComputerPilot.png)

This time, the autopilot is implemented in a StateFlow chart. 
Using telemetry information and the drone's front camera, the system must complete the entire circuit on its own (without human intervention).

![Autopilot StateFlow chart](./img/autopilot.png)

This starting prototype incorporates basic intelligence, which does not cover many situations that may arise:

1) The drone memorizes the initial position and turn the rotors on.
2) The drone heads to the center of the tatami, where it starts to rotate. As soon as it detects the red frame, it will move directly toward it. In its proximity, it will start circling around until it is in front of the opening. Then it will move forward for a certain amount of time.
3) It will then repeat this behavior for the green and blue frames.
4) Finally, it will move directly to the starting position, where it will turn off the rotors.

![Autopilot red frame state](./img/RedFrame_state.png)

Competition participants are tasked with enhancing this proposal, making it robust against unexpected situations.



## Visit program at I3A

A variant of the previously described _DroneChallenge_ competition is the drone-related activity that is part of the _Visiting Program for Primary and Secondary School Students_ to the **Albacete Research Institute of Informatics** ([I3A](https://www.uclm.es/centros-investigacion/i3a?sc_lang=en)).

We run the command `gazebo I3A.world` to open the appropiate world, which displays a scenario similar to the previous one, with the exception that it includes 5 drones. 
This time, up to 4 young pilots compete against the autonomous drone to cross the three frames and land at the base located at the other end of the tatami!

In the background, behind the tatami, is the I3A building.

![I3A](./img/I3A_world.png)

In the Matlab environment, the corresponding scripts and models are in the `navsim/matlab/operators/I3Avisit` directory.

![Pilots manager](./img/pilots_manager.png)
