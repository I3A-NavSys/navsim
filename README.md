# utrafman_ros2
Repositorio para migrar el simulador de ROS1 a ROS2

# Versiones

En esta [página](https://gazebosim.org/docs/fortress/ros_installation) se indican las versiones ROS Gazebo compatibles:

Optamos por la combinación ROS 2 Humble (LTS) / GZ Fortress (LTS)



# Configuración de máquina

Instalar la plataforma VMware Workstation 17.0.2 Player

Crear una máquina virtual con
- 8 procesadores (que ejecutarán Gazebo)
- 32MB RAM
- HD de 50GB
- Activar aceleración 3D (asignando 8GB)
- Conectar con un DVD incluyendo la ISO Desktop Image UBUNTU 22.04.3 LTS (Jammy Jellyfish)

# Instalación de UBUNTU
1. Configurar teclado en español Windows
2. Instalar
	- La versión normal
	- Descargar actualizaciones mientras se instala
	- No instalar contenido de terceros
3. Configurar en español (renombrar carpetas)
4. Dejar que el sistema se actualice y reinicie
5. Abrir “Ubuntu software”
	- En la lengüeta de **Actualizaciones**, actualizar todo.
 	- Instalar Visual Studio Code
 	- Reiniciar

# Instalar ROS 2 Humble Hawksbill (LTS)
Seguir los pasos de este tutorial:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

en el paso de asignar la fuente, podemos agregarla al bash:

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

# Instalar Gazebo

Instalar Gazebo Fortress (LTS) siguiendo los pasos de este [tutorial](https://gazebosim.org/docs/fortress/install_ubuntu).

Probar el comando `ign gazebo lights.sdf`.

En la máquina virtual puede que no se reproduzca bien. 
En tal caso ejecutarlo con ogre (en lugar de ogre2)
`ign gazebo lights.sdf --render-engine ogre`


Si Gazebo no se reproduce bien, introducir este codigo en bashrc

```bash
# Configure the Mesa Environment
# https://docs.mesa3d.org/envvars.html

# Core Mesa environment variables
export MESA_DEBUG=1
export MESA_GL_VERSION_OVERRIDE=4.1
export MESA_GLSL_VERSION_OVERRIDE=410
export MESA_EXTENSION_OVERRIDE="\
  -GL_ARB_buffer_storage \
  -GL_ARB_multi_draw_indirect \
  -GL_ARB_texture_buffer_range \
  -GL_ARB_compute_shader \
  -GL_ARB_ES3_compatibility \
  "
```


# Probar los tutoriales básicos:

https://gazebosim.org/docs/fortress/building_robot
https://gazebosim.org/docs/fortress/moving_robot


	

