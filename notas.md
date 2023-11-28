# primero creamos el espacio de trabajo en el lugar deseado
cd ~/code/utrafman_ros2

mkdir -p utrafman_ws/src
cd utrafman_ws
colcon build --symlink-install

# despues creamos paquetes dentro
cd src
ros2 pkg create utrafman_pkg


# compilar de nuevo
cd ~/code/utrafman_ros2
colcon build --symlink-install

# ya indica un paquete compilado
echo "source ~/code/utrafman_ros2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# observamos nuestro paquete en la lista reconocida por ROS2
ros2 pkg list

# de momento no tiene nada ejecutable
ros2 pkg list

