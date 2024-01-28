
# NavSim

Update 15/01/2024

![NavSim](./doc/img/navsim.png)


NAVSIM is a **U-Space** service development tool. It provides:

- Real-time simulation of 3D urban airspace (executed in the Gazebo 11 _Classic_ environment).

- Communication support between different services (implemented in ROS2).

- A comprehensive set of tools (developed in Matlab® R2023b) to:

  - Define **drone operators** capable of generating flight operation needs.
  - Generate **flight plans** for the execution of these operations.
  - Detect and resolve conflicts between flight plans.
  - Deploy **UAVs** in the scenario capable of executing a flight plan provided by their operator.
    ![100 drones](./doc/tutos/tuto2/img/100drones.png)
 

  - Monitor the execution of flight plans.
    ![Matlab tools example](./doc/img/tool_example.png)

## Resources

- [Installation](./doc/install.md)
- [Tutorial 1:](./doc/tutos/tuto1/tuto1.md) Running your first simulation with Matlab connection
- [Tutorial 2:](./doc/tutos/tuto2/tuto2.md) Running 100 drones
- [Tutorial 3:](./doc/tutos/tuto3/tuto3.md) Executing time/position-based flight plans
