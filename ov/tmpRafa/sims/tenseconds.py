# Importar las librerías necesarias
import time
from omni.isaac.kit import SimulationApp

# Iniciar la aplicación de simulación
simulation_app = SimulationApp({"headless": False})  # Cambiar a True si no necesitas interfaz gráfica

# Importar módulos necesarios de Isaac Sim
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core import SimulationContext
from omni.isaac.core.utils.stage import open_stage

# Obtener el contexto de simulación
simulation_context = SimulationContext()

# Ruta a tus activos o escena (puede variar según tu configuración)
assets_root_path = get_assets_root_path()
scene_path = assets_root_path + "/Isaac/Environments/Simple_Warehouse/warehouse.usd"

# Cargar la escena
open_stage(scene_path)

# Iniciar la simulación
simulation_context.play()

# Esperar 10 segundos
time.sleep(10)

# Detener la simulación
simulation_context.stop()

# Cerrar la aplicación de simulación
simulation_app.close()
