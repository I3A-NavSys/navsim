#primero creamos el espacio de trabajo

cd ~/code

mkdir -p ros2_workspace/src
cd ros2_workspace
colcon build --symlink-install

# despues creamos paquetes dentro
cd src
ros2 pkg create my_package


#compilar de nuevo
cd ../..
colcon build --symlink-install

#ya indica un paquete compilado