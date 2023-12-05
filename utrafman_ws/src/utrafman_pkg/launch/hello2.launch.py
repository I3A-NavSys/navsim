import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Obtén la ruta al mundo de Gazebo que deseas cargar
    gazebo_world_file = os.path.join(get_package_share_directory('utrafman_pkg'), 'worlds', 'hello.world')

    # Crea la descripción del lanzamiento
    ld = LaunchDescription([
        # Lanza el servidor de Gazebo con tu mundo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', gazebo_world_file],
            output='screen'
        ),
        
        # Aquí puedes añadir nodos adicionales si es necesario
    ])

    return ld

