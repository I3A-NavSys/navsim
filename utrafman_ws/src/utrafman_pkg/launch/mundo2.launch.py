# my_gazebo_package/launch/gazebo_launch.py

# /ruta/a/tu/paquete/launch/mi_lanzamiento_gazebo.py

import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity', 'world', '-file', '/home/usuario/code/utrafman_ros2/utrafman_ws/src/utrafman_pkg/worlds/tatami.world'],
        ),
    ])
