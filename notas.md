primero creamos el espacio de trabajo en el lugar deseado
```bash
cd ~/code/utrafman_ros2
mkdir -p utrafman_ws/src
cd utrafman_ws
colcon build --symlink-install
```

despues creamos paquetes dentro
```bash
cd src
ros2 pkg create utrafman_pkg
```

compilar de nuevo
```bash
cd ~/code/utrafman_ros2
colcon build --symlink-install
```

ya indica un paquete compilado. ahora lo añadimos al bash
```bash
echo "source ~/code/utrafman_ros2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

observamos nuestro paquete en la lista reconocida por ROS2
```bash
ros2 pkg list
```

de momento no tiene nada ejecutable, pero lo reconoce
```bash
ros2 pkg list
```

añadimos un plugin de mundo. 

compilamos de nuevo desde el raiz

añadimos el directorio resultante al path de Gazebo, escribiendo en bashrc:
```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/code/utrafman_ros2/utrafman_ws/install/utrafman_pkg/lib/utrafman_pkg/
```

finalmente, definimos un mundo invocando dicho plugin. Lo abrimos en Gazebo y vemos el resultado.
```bash
cd ~/code/utrafman_ros2/utrafman_ws/src/utrafman_pkg/worlds/
clear; gazebo --verbose hello.world
```

