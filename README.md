# utrafman_ros2
Repositorio para migrar el simulador de ROS1 a ROS2.
Actualización 23/11/2023

# Versiones

Matlab R2023b [es compatible](https://es.mathworks.com/help/ros/gs/ros-system-requirements.html) 
con ROS2 versión Humble Hawksbill (LTS), 
que [es compatible](https://gazebosim.org/docs/fortress/ros_installation) 
con Gazebo versión GZ Fortress (LTS). 
Sin embargo, esta versión está resultando muy inestable (en una máquina virtual). 

Mathworks [recomienda](https://www.mathworks.com/support/product/robotics/ros2-vm-installation-instructions-v9.html)
usar Gazebo 11 incluso con ROS2. Vamos a intentarlo.

# Configuración de máquina

Instalar la plataforma VMware Workstation 17.5.0 Player.
Instalar el teclado

Crear una máquina virtual con
- 8 procesadores (que ejecutarán Gazebo)
- 32MB RAM
- HD de 50GB
- Activar aceleración 3D (asignando 8GB)
- Conectar con un DVD incluyendo la ISO Desktop Image UBUNTU 22.04.3 LTS (Jammy Jellyfish). Descargarla de [aquí](https://releases.ubuntu.com/jammy/).

# Instalación de UBUNTU
1. Configurar teclado en español Windows
2. Instalar
	- La versión normal
	- Descargar actualizaciones mientras se instala
	- No instalar contenido de terceros
3. Configurar en español
	- Inicio/Settings/Region & Language
 	- Renombrar carpetas
5. Dejar que el sistema se actualice y reinicie
6. Abrir “Ubuntu software”. En la lengüeta de **Actualizaciones**, actualizar todo y reiniciar.



# Instalar ROS2 Humble Hawksbill (LTS)

Seguir los pasos de este [tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).
En el paso de asignar la fuente, podemos agregarla al bash:

```bash
source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

# Instalar Gazebo

Instalar Gazebo Fortress (LTS) siguiendo los pasos de este [tutorial](https://gazebosim.org/docs/fortress/install_ubuntu).

Lanzar un ejemplo
```bash
ign gazebo lights.sdf
```

En la máquina virtual puede que no se reproduzca bien (la imagen parpadea y no se muestran los objetos). En tal caso ejecutarlo renderizando con ogre (en lugar de ogre2)
```bash
ign gazebo lights.sdf --render-engine ogre
```

Instalar puente ROS-Gazebo
```bash
sudo apt-get install ros-humble-ros-ign-bridge
```

Probar los tutoriales básicos:

[building_robot](https://gazebosim.org/docs/fortress/building_robot)

[moving_robot](https://gazebosim.org/docs/fortress/moving_robot)


# Instalar Git

```bash
sudo apt update
sudo apt install git
git --version
```


# Instalar Visual Studio Code
1. Abrir _Ubuntu software_ e instalar **Visual Studio Code**
2. Instalar extensión **Spanish Language Pack for Visual Studio Code**
3. Instalar extensión **markdownlint**
4. Instalar extensión **GitHub Pull Request and Issues**
	- En Accounts (abajo a la izquierda, encima de la rueda dentada) pulsar _Authorize GitHub for VS Code_


# Repositorio UTRAFMAN2

Clonar el repositorio desde GitHub. Utilizar VS Code, o directamente en consola:

```bash
git clone https://github.com/I3A-NavSys/utrafman_ros2
```

Probar un mundo de ejemplo

```bash
cd utrafman_ros2/worlds/
ign gazebo -v 0 tatami.world --render-engine ogre
```

# MATLAB

Matlab tiene una máquina virtual con todo instalado: ROS Noetic/Humble Gazebo 11
[ROS Noetic and ROS 2 Humble and Gazebo](https://www.mathworks.com/support/product/robotics/ros2-vm-installation-instructions-v9.html)


