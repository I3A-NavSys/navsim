# utrafman_ros2
Repositorio para migrar el simulador de ROS1 a ROS2



En esta [página](https://gazebosim.org/docs/fortress/ros_installation) se indican las versiones ROS Gazebo compatibles:

Optamos por la combinación ROS 2 Humble (LTS) / GZ Fortress (LTS)



# Configuración de máquina

Instalar la plataforma VMware Workstation 17.0.2 Player

Crear una máquina virtual con
-	8 procesadores (que ejecutarán Gazebo)
-	32MB RAM
-	HD de 50GB
-	Activar aceleración 3D (asignar 8GB)
-	Conectar con un DVD incluyendo la ISO de UBUNTU 22.04.3 LTS (Jammy Jellyfish)

# Instalación de UBUNTU
1.	Configurar teclado en español Windows
2.	Instalar la versión Desktop Image
  -	Instalación normal
  - Descargar actualizaciones mientras se instala
  - Instalar contenido de terceros (video drivers…)
3.	Configurar en español (renombrar carpetas)
4.	Dejar que el sistema se actualice y reinicie
5.	Dejar que las aplicaciones se actualicen y reiniciar
  -	Abrir “Ubuntu software” e ir a la lengüeta de actualizaciones. Actualizar todo.

# Instalar ROS 2 Humble Hawksbill (LTS)
Seguir los pasos de este tutorial:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

en el paso de asignar la fuente, podemos agregarla al bash:

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

# Instalar Gazebo

Se debe usar Gazebo Fortress (LTS) con ROS2 Humble. Seguir los pasos de este tutorial:
`https://gazebosim.org/docs/fortress/install_ubuntu`

Probar el comando `ign gazebo`

# Probar los tutoriales básicos:

https://gazebosim.org/docs/fortress/building_robot
https://gazebosim.org/docs/fortress/moving_robot


	

