# 01: Running your first simulation with Matlab connection

## Launch the scenario

First, we open a scenario in the Gazebo simulator:

```bash
cd
cd code/navsim/ws/src/navsim_pkg/worlds
gazebo DroneChallenge.world
```
An environment should open with a gaming area of 10x10 meters. On a white base (the _vertiport_), there is a quadcopter (called _abejorro1_). Floating in the air, you could see three frames colored red, green, and blue, respectively.

![DroneChallenge](./img/DroneChallenge.png 'Drone Challenge scenario. :size=600px')


In a new terminal, we can verify that ROS is running correctly. The command `$ros2 node list` shows four active nodes at the moment.


## Check world node

The node **/World** is associated with the loaded scenario. Using the `ros2 topic list` command, we observe that it generates a topic **/World/Time** where we can check the simulation time. With these commands, we can determine the structure of the transmitted message:

```bash
ros2 topic type /World/Time
ros2 interface show builtin_interfaces/msg/Time
```
Then, with the command `ros2 topic echo /World/Time`, we observe that the data refreshes 10 times per second.


With the command `ros2 service list`, we check that there are (among others) two services associated with this node, named **/World/DeployModel** and **/World/RemoveModel** respectively. The first command is more complex to use and will be covered in a later tutorial. The second command allows us to remove an element from the scenario.
```bash
ros2 service type /World/RemoveModel 
ros2 interface show navsim_msgs/srv/RemoveModel
```
We see that we can call the service simply by indicating the name of the object we want to remove. As an example, let's remove the landing pad with the command:
```bash
ros2 service call /World/RemoveModel navsim_msgs/srv/RemoveModel "{name: 'vertiport'}"
```
We see that the landing pad disappears, and the drone falls to the ground.
Before proceeding, close Gazebo and let's load the original scenario again to recover the vertiport.

```bash
gazebo DroneChallenge.world
```


## Check UAV node

Each UAV generates its own ROS node to interact with the environment. In this case, the node for the quadcopter is named **/abejorro1**.
This node manages the transmission of telemetry information and the reception and execution of control commands.


### Telemetry

All existing UAVs will publish their telemetry information on the **/UAV/Telemetry** topic. With the command `ros2 topic info /UAV/Telemetry`, we can see that there is a node transmitting messages on this topic. Using the command `ros2 interface show navsim_msgs/msg/Telemetry`, we can see that the message contains:
- Aircraft identifier
- Position and orientation
- body linear and angular velocities
- Simulation time when the data was generated

With the command `ros2 topic echo /UAV/Telemetry` we check that this data is updated once per second.

### Navigation

Each UAV has its own control topic, unlike telemetry transmission where all UAVs share a single topic.
Estudiemos el tópico con estos comandos:
```bash
ros2 topic list
ros2 topic type /UAV/abejorro1/RemoteCommand
ros2 interface show navsim_msgs/msg/RemoteCommand 
```
We observe that the control topic is named **/UAV/abejorro1/RemoteCommand**. The command includes:
- Aircraft identifier (only required in cases where the topic refers to a swarm)
- Activation/deactivation of rotors
- Reference linear velocity (expressed in horizon axes)
- Reference angular velocity (only around the vertical axis)
- Time of validity of the command.

We can move the quapcorter publishing the following commands:
```bash
ros2 topic pub -1 /UAV/abejorro1/RemoteCommand navsim_msgs/msg/RemoteCommand "{'on': true, 'vel': {'linear': {z: 1}}, 'duration': {'sec': 1}}"
ros2 topic pub -1 /UAV/abejorro1/RemoteCommand navsim_msgs/msg/RemoteCommand "{'on': true, 'vel': {'linear': {x: 1}, 'angular': {z: 1}}, 'duration': {'sec': 6}}"
ros2 topic pub -1 /UAV/abejorro1/RemoteCommand navsim_msgs/msg/RemoteCommand "{'on': false}"
```

## Check UAV cameras

This quadcopter has two cameras. One camera is real and is mounted on the front of the fuselage. 
The other camera is virtual and hovers at a certain height behind the aircraft. 
Each camera generates its own node for image transmission.


## Launch Pilot interface

> Warning: This step requires having a joystick connected to the computer running Matlab / Simulink. If you don't have one, you can skip this section and proceed to the next one.
>
> Este tutorial se ha realizado con un joystick modelo **Thrustmaster T.Flight Stick X**

In the Matlab environment, navigate to the `navsim/matlab/operators/DroneChallenge` directory. From there, open the `HumanPilot.slx` model.

![Drone Challenge human pilot](./img/HumanPilot.png 'Drone Challenge human pilot. :size=600px')

Press the **Run** button to execute the model. After compilation, two additional windows will open. One displays the video image captured by the drone's front camera. The other shows the image from a virtual camera flying behind the drone, designed to assist the human pilot. You can rearrange and resize these windows on the screen as desired or even close them.

![Drone Challenge onboard camera](./img/onboardCAM.png 'Drone Challenge onboard camera. :size=600px')

![Drone Challenge follow camera](./img/followCAM.png 'Drone Challenge follow camera. :size=600px')


The model displays a speedometer at the top right and a maximum speed limiter at the bottom right. It is recommended to limit the speed to below 2 m/s in the initial attempts.

On the left, there is a joystick diagram. At its base, there is a circular LED that indicates whether the quadcopter motors are on (green) or off (red).
- To turn on the motors, press button 1 on the joystick (index finger). Initially, the drone stabilizes, and it may not be immediately apparent that the motors are on. Check the LED if in doubt.
- To turn off the motors, press button 2 on the joystick (thumb). The drone will descend to the ground if it was hovering in the air.

With the motors on, the drone stabilizes automatically. Then, you can:
- ascend/descend: employing the thrust lever.
- do planar movement: moving the stick.
- rotate: rotating the stick.




