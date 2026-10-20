import importlib
from os import path
import sys

current_file_path = path.dirname(__file__)
project_root_path = path.abspath(path.join(current_file_path, "../../.."))

if project_root_path not in sys.path:
    sys.path.insert(0, project_root_path)

# En Isaac Sim, si mavlink ya estaba cargado previamente en memoria antes de añadir
# MavManualControlMsg, los módulos en sys.modules pueden estar desactualizados.
# Forzamos recarga limpia para garantizar que todas las clases existan.
for mod_name in ("mavlink.dialect", "mavlink.udp_manager", "mavlink"):
    if mod_name in sys.modules:
        try:
            importlib.reload(sys.modules[mod_name])
        except Exception:
            pass

from .extension import *

