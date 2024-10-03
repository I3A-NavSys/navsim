from pxr import Usd, Gf
import sys, os
import random
import omni
import omni.usd
import omni.ext



##############################################################################

# Adding root 'ov' folder to sys.path
import sys, os

current_path = os.path.abspath(os.path.dirname(__file__))
while True:
    if os.path.basename(current_path) == 'ov':
        project_root_path = current_path
        break
    parent_path = os.path.dirname(current_path)
    if parent_path == current_path:
        raise RuntimeError("No se encontró el directorio 'ov' en la ruta.")
    current_path = parent_path
# print(f"Directorio raíz del proyecto: {project_root_path}")

if project_root_path not in sys.path:
    sys.path.append(project_root_path)



##############################################################################
# Abre archivo USD en el entorno de Isaac Sim
usd_file_path = os.path.join(project_root_path, "assets/worlds/generated_city.usda")
# print(f"Directorio: {usd_file_path}")

context = omni.usd.get_context()
if not context.open_stage(usd_file_path):
    raise RuntimeError(f"Error: No se pudo abrir el archivo USD en {usd_file_path}")

stage: Usd.Stage = context.get_stage()
# print(stage.ExportToString())




##############################################################################
# Creamos vertipuertos en posiciones predefinidas
# 


# Seleccionamos archivo USD
VP_usd_file_path = os.path.join(project_root_path, "assets/vertiports/vertiport.usd")
# print(f"Directorio: {UAV_usd_file_path}")
if not os.path.exists(VP_usd_file_path):
    raise FileNotFoundError(f"Error: No se encontró el archivo USD {VP_usd_file_path}")


# Importamos el escenario del archivo USD
VP_stage = Usd.Stage.Open(VP_usd_file_path)
if not VP_stage:
    raise RuntimeError(f"Error: No se pudo abrir el archivo USD {VP_usd_file_path}")


# Obtenemos el prim raíz del vertipuerto
VP_root_prim = VP_stage.GetDefaultPrim()
if not VP_root_prim:
    raise RuntimeError(f"Error: No se pudo obtener el prim raíz de {VP_usd_file_path}")


# Añadimos los vertipuertos al escenario

#                x        y        z       rz
portsLoc = [
            [ -190.00, -119.00, +048.00,   3.14/4],
            [ -152.00, -106.00, +049.00,   3.14/4],
            [ -134.00, -190.00, +048.00,   0     ],
            [ -092.00, -144.00, +041.00,   0     ],
            [ -074.00, -100.00, +043.00,   0     ],
            [ -073.00, +216.00, +027.00,   0     ],
            [ -007.00, +015.00, +043.00,   0     ],
            [ +060.00, +131.00, +032.00,   0     ],
            [ +180.00, +033.00, +050.00,   0     ],
            [ +186.00, -081.00, +050.00,   3.14/2],
            [ -200.00, +157.00, +044.00,   3.14/2],
            [ -200.00, +020.00, +042.00,   3.14/2],
            [ +186.00, +195.00, +039.00,   3.14/2],
            [ +126.00, -189.00, +039.00,   3.14/2]
        ]

for i, loc in enumerate(portsLoc):
    VP_prim_path = f"/World/VPs/VP{i}"

    # Cargar el payload del vertipuerto
    VP_prim = stage.DefinePrim(VP_prim_path)
    VP_prim.GetPayloads().AddPayload(VP_usd_file_path, VP_root_prim.GetPath())

    # Establecer la posición del vertipuerto
    posX, posY, posZ, rotZ = loc
    VP_prim.GetAttribute("xformOp:translate").Set((Gf.Vec3d(posX, posY, posZ)))
    rotZq = Gf.Quatd(Gf.Rotation(Gf.Vec3d(0, 0, 1), rotZ).GetQuat())
    VP_prim.GetAttribute("xformOp:orient").Set(rotZq)





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


# Añadimos los UAVs al escenario

fleet_scope_path = "/Fleet"
fleet_scope = stage.DefinePrim(fleet_scope_path, "Scope")

for i, loc in enumerate(portsLoc):
    UAV_prim_path = f"/Fleet/UAV{i}"

    # Cargar el payload del dron
    UAV_prim = stage.OverridePrim(UAV_prim_path)
    UAV_prim.GetPayloads().AddPayload(UAV_usd_file_path, UAV_root_prim.GetPath())

    # Establecemos la posición del UAV
    posX, posY, posZ, rotZ = loc
    posZ += 0.2
    UAV_prim.GetAttribute("xformOp:translate").Set((Gf.Vec3d(posX, posY, posZ)))

    # Establecemos la orientación del UAV
    rotZ = random.uniform(-180, 180)  
    rotZq = Gf.Quatd(Gf.Rotation(Gf.Vec3d(0, 0, 1), rotZ).GetQuat())
    UAV_prim.GetAttribute("xformOp:orient").Set(rotZq)



##############################################################################
# Iniciar la simulación
import omni.timeline
timeline = omni.timeline.get_timeline_interface()
timeline.play()


