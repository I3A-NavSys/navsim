# my_gazebo_package/launch/gazebo_launch.py

import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='gazebo_ros',
            output='screen',
            arguments=['-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            parameters=[{'use_sim_time': 'true'}],
            remappings=[
                ('/gazebo', '/gazebo'),
                ('/gazebo/link_states', '/gazebo/link_states'),
                ('/gazebo/model_states', '/gazebo/model_states'),
                ('/gazebo/parameter_descriptions', '/gazebo/parameter_descriptions'),
                ('/gazebo/parameter_updates', '/gazebo/parameter_updates'),
            ],
            name='gazebo',
        ),
        # Add your world file path below
        # Replace '/path/to/your/world_file.world' with the actual path
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'world', '-file', '/path/to/your/world_file.world'],
            output='screen',
            name='spawn_entity',
        ),
    ])
