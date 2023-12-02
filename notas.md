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
ros2 pkg create utrafman_msgs
```

compilar de nuevo. ya debe indicar dos paquetes compilados. 
```bash
cd ~/code/utrafman_ros2
colcon build --symlink-install
```

Ahora configuramos bash para que nuestro espacio de trabajo actúe como ROS overlayer, dejando a Humble como ROS underlayer:
```bash
echo "source ~/code/utrafman_ros2/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

observamos nuestros paquetes en la lista reconocida por ROS2
```bash
ros2 pkg list | grep utrafman
```

cosas utiles para comprobar la generación de mensajes ROS:
```bash
ros2 interface list | grep utrafman
ros2 interface show utrafman_msgs/msg/Operator
ros2 service call /utrafman/Test utrafman_msgs/srv/Test '{a: 2, b: 3}'
ros2 service call /utrafman/DeployUAV utrafman_msgs/srv/DeployUAV "{model_sdf: 'tu_modelo_sdf_aqui'}"
```

de momento no tiene nada ejecutable, pero lo reconoce
```bash
ros2 pkg list
```

añadimos un plugin de mundo. 

compilamos de nuevo desde el raiz

añadimos las librerias resultantes al path de Gazebo, escribiendo en bashrc:
```bash
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/code/utrafman_ros2/utrafman_ws/install/utrafman_pkg/lib/utrafman_pkg/
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/code/utrafman_ros2/utrafman_ws/install/utrafman_msgs/lib/
```


finalmente, definimos un mundo invocando dicho plugin. Lo abrimos en Gazebo y vemos el resultado.
```bash
cd ~/code/utrafman_ros2/utrafman_ws/src/utrafman_pkg/worlds/
clear; gazebo --verbose hello.world
```

