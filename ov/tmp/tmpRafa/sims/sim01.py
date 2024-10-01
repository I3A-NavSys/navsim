from pxr import Usd, Gf

import sys, os
import random
import omni
import omni.usd
import omni.ext

##############################################################################

# Adding root 'ov' folder to sys.path
import sys, os
project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)

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



num_UAVs = 20
for i in range(num_UAVs):
    UAV_prim_path = f"/World/UAV{i}"

    # Cargar el payload del dron
    UAV_prim = stage.OverridePrim(UAV_prim_path)
    UAV_prim.GetPayloads().AddPayload(UAV_usd_file_path, UAV_root_prim.GetPath())


    # Establecemos la posición del UAV
    pos_atr = UAV_prim.GetAttribute("xformOp:translate")
    x = random.uniform(-4, -2)
    y = random.uniform( 1,  3)
    z = random.uniform( 1,  2)  
    pos_atr.Set(Gf.Vec3d(x, y, z))

    # rot_atr = UAV_prim.GetAttribute("xformOp:rotateXYZ")
    # ori_atr = UAV_prim.GetAttribute("xformOp:orient")





##############################################################################
# Iniciar la simulación
import omni.timeline
timeline = omni.timeline.get_timeline_interface()
timeline.play()


