# Importa las bibliotecas necesarias
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Ruta al archivo de mundo
    world_file_path = "../worlds/hello.world"

    # Define el nodo de Gazebo
    gazebo_node = Node(
        package='gazebo_ros',
        executable='gazebo_ros',
        name='gazebo',
        output='screen',
        arguments=['-s', 'libgazebo_ros_factory.so', world_file_path]
        # arguments=['-s', 'libgazebo_ros_diff_drive.so', world_file_path]
    )

    # Retorna la descripción del lanzamiento con el nodo de Gazebo
    return LaunchDescription([gazebo_node])
