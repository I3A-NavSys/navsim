# ejecutammos este script desde la extensión de contenidos de Isaac Sim
# podemos conectar al debugger de VSCode para depurar el código

# creamos algunos objetos en pantalla...

print("ESTE ES UN MENSAJE POR CONSOLA")

from pxr import Usd, UsdGeom
import omni.usd
import numpy as np

stage = omni.usd.get_context().get_stage()

from omni.isaac.core.objects import GroundPlane
plane = GroundPlane(prim_path="/World/GroundPlane", z_position=0)



from omni.isaac.core.objects import DynamicCuboid
prim = DynamicCuboid(
	name="caja",
	prim_path="/World/CUBOROJO", 
	position=np.array([0.0, 0.0, 10.0]), 
	color=np.array([1.0, 0.0, 0.0]), 
	mass=1.0)
prim

import omni
omni.timeline.get_timeline_interface().play()

import omni
from pxr import Usd, UsdGeom, Gf
import random


# Obtener el contexto de USD y la etapa actual
stage = omni.usd.get_context().get_stage()

# Definir el número de robots
num_robots = 5

# Crear los robots en posiciones aleatorias
for i in range(num_robots):
    # Generar una posición aleatoria
    x = random.uniform(-10, 10)
    y = random.uniform(-10, 10)
    z = 5  

    # Definir la ruta del prim del robot
    robot_prim_path = f"/World/Robot_{i}"

    # Crear un prim Xform para el robot
    robot_prim = UsdGeom.Xform.Define(stage, robot_prim_path)

    # Establecer la posición del robot
    robot_prim.AddTranslateOp().Set(Gf.Vec3f(x, y, z))

    # Añadir una geometría simple al robot (un cubo, por ejemplo)
    UsdGeom.Cube.Define(stage, f"{robot_prim_path}/Body")

# Guardar el escenario
stage.GetRootLayer().Save()
