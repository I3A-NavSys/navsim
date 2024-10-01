from pxr import Usd, UsdGeom, Gf


import sys, os
import random
import omni
import omni.usd
import numpy as np



##############################################################################
# Adding root 'ov' folder to sys.path
project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
# print(f"Directorio raíz del proyecto: {project_root_path}")
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

##############################################################################
# Abre archivo USD en el entorno de Isaac Sim
usd_file_path = os.path.join(project_root_path, "tmp/tmpRafa/pruebas/balancin.usd")
# print(f"Directorio: {usd_file_path}")

context = omni.usd.get_context()
if not context.open_stage(usd_file_path):
    raise RuntimeError(f"Error: No se pudo abrir el archivo USD en {usd_file_path}")

stage: Usd.Stage = context.get_stage()
# print(stage.ExportToString())




##############################################################################
# Creamos UAVs en posiciones aleatorias
# 

# Seleccionamos archivo USD
UAV_usd_file_path = os.path.join(project_root_path, "fleet/UAM_minidrone/UAM_minidrone.usd")
# print(f"Directorio: {UAV_usd_file_path}")
if not os.path.exists(UAV_usd_file_path):
    raise FileNotFoundError(f"Error: No se encontró el archivo USD del UAV en {UAV_usd_file_path}")

# Importamos el escenario del archivo USD
UAV_stage = Usd.Stage.Open(UAV_usd_file_path)
if not UAV_stage:
    raise RuntimeError(f"Error: No se pudo abrir el archivo USD del UAV en {UAV_usd_file_path}")

# Obtenemos el prim raíz del UAV
UAV_root_prim = UAV_stage.GetDefaultPrim()
if not UAV_root_prim:
    raise RuntimeError(f"Error: No se pudo obtener el prim raíz del UAV en {UAV_usd_file_path}")



num_UAVs = 1
for i in range(num_UAVs):

    # Creamos un prim Xform para el UAV
    UAV_prim_path = f"/World/UAV{i}"
    UAV_prim = UsdGeom.Xform.Define(stage, UAV_prim_path)

    # # Establecemos la posición del UAV
    # x = random.uniform(1, 3)
    # y = random.uniform(1, 3)
    # z = random.uniform(1, 3)  
    # UAV_prim.AddTranslateOp().Set(Gf.Vec3d(x, y, z))

    # Clonar el prim del robot al escenario principal como payload
    UAV_prim = stage.OverridePrim(UAV_prim_path)
    UAV_prim.GetPayloads().AddPayload(UAV_usd_file_path, UAV_root_prim.GetPath())





# from operators.USpaceOperator import USpaceOperator
# from planners.Waypoint import Waypoint
# from planners.FlightPlan import FlightPlan

# # Crear una instancia de UspaceOperator
# op = USpaceOperator("Operador Uno")

# # Acceder al atributo name
# print(op.name)  # Imprime: Operador Uno



# # Ejemplo de uso
# waypoint = Waypoint(label='WP1', t=10, pos=[1, 2, 3], vel=[0.1, 0.2, 0.3])
# print(waypoint.label)
# print(waypoint.t)
# print(waypoint.pos)
# print(waypoint.vel)
# print(waypoint.mandatory)
