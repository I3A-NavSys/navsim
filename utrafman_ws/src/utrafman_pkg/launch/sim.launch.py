
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    ld = LaunchDescription()

    world = LaunchConfiguration('world', default='home')
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='home',
        description='World name')
    ld.add_action(declare_world_cmd)



    tiago_gazebo_dir = get_package_share_directory('tiago_gazebo')
    tiago_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tiago_gazebo_dir, 'launch', 'tiago_gazebo.launch.py')),
        launch_arguments={
          'world_name': world,
          'arm': 'no-arm'
        }.items())
    ld.add_action(tiago_sim_cmd)




    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    ld.add_action(declare_use_sim_time_cmd)




    return ld
