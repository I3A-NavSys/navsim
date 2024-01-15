# Development notes


First, we create the workspace in the desired location:

```bash
cd ~/code/navsim
mkdir -p ws/src
cd ws
colcon build --symlink-install
```

Next, we create packages inside:

```bash
cd src
ros2 pkg create navsim_pkg
ros2 pkg create navsim_msgs
```

Compile again. It should now indicate that two packages have been built.

```bash
cd ~/code/navsim
colcon build --symlink-install
```

Now, we set up bash to make our workspace act as a ROS overlay, leaving Humble as a ROS underlay:

```bash
echo "source ~/code/navsim/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

We observe our packages in the list recognized by ROS2:

```bash
ros2 pkg list | grep navsim
```

Here is a list of useful commands to check the generation of ROS messages:

```bash
ros2 pkg list | grep navsim
ros2 interface list | grep navsim
ros2 interface show navsim_msgs/srv/DeployModel
ros2 service call /World/DeployModel navsim_msgs/srv/DeployModel "model_sdf: 'your model here'"
ros2 topic pub /RemotePilot navsim_msgs/msg/FlyCommand "{'on': true, 'vel': { 'linear': { 'x': 0.1, 'y': 0.0, 'z': 0.0 }, 'angular': { 'x': 0.0, 'y': 0.0, 'z': 0.0 } }, 'duration': { 'sec': 10, 'nanosec': 0 }}"
```
