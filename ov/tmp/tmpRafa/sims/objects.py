# Ejecutamos este script desde la extensión de contenidos de Isaac Sim
# Podemos conectar al debugger de VSCode para depurar el código

##############################################################################

import sys
import os
if os.name == 'nt':
    os.system('cls')
else:
    os.system('clear')

# Adding root 'ov' folder to sys.path
import sys, os
project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

##############################################################################




from pxr import Usd, UsdGeom, Gf
import random
import omni
import omni.usd
import numpy as np






##############################################################################

# Abre archivo USD en el entorno de Isaac Sim
current_dir = os.path.dirname(__file__)
print(f"Directorio actual: {current_dir}")
usd_file_path = os.path.join(current_dir, "../pruebas/balancin.usd")

context = omni.usd.get_context()
if context.open_stage(usd_file_path):
    print(f"Archivo USD cargado correctamente en el escenario: {usd_file_path}")
else:
    raise RuntimeError(f"Error: No se pudo abrir el archivo USD en {usd_file_path}")

stage: Usd.Stage = context.get_stage()
print(stage.ExportToString())
##############################################################################








# from omni.isaac.core.objects import GroundPlane
# plane = GroundPlane(prim_path="/World/GroundPlane", z_position=0)



# from omni.isaac.core.objects import DynamicCuboid
# prim = DynamicCuboid(
# 	name="caja",
# 	prim_path="/World/CUBOROJO", 
# 	position=np.array([0.0, 0.0, 10.0]), 
# 	color=np.array([1.0, 0.0, 0.0]), 
# 	mass=1.0)
# prim




# omni.timeline.get_timeline_interface().play()



# # Obtener el contexto de USD y la etapa actual
# stage = omni.usd.get_context().get_stage()

# # Definir el número de robots
# num_robots = 5

# # Crear los robots en posiciones aleatorias
# for i in range(num_robots):
#     # Generar una posición aleatoria
#     x = random.uniform(-10, 10)
#     y = random.uniform(-10, 10)
#     z = 5  

#     # Definir la ruta del prim del robot
#     robot_prim_path = f"/World/Robot_{i}"

#     # Crear un prim Xform para el robot
#     robot_prim = UsdGeom.Xform.Define(stage, robot_prim_path)

#     # Establecer la posición del robot
#     robot_prim.AddTranslateOp().Set(Gf.Vec3f(x, y, z))

#     # Añadir una geometría simple al robot (un cubo, por ejemplo)
#     UsdGeom.Cube.Define(stage, f"{robot_prim_path}/Body")

# # Guardar el escenario
# # stage.GetRootLayer().Save()
