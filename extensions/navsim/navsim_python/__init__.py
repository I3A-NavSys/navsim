import importlib
from os import path
import sys

current_file_path = path.dirname(__file__)
project_root_path = path.abspath(path.join(current_file_path, "../../.."))

if project_root_path not in sys.path:
    sys.path.insert(0, project_root_path)

# En Isaac Sim, forzamos recarga limpia de submódulos para que cualquier cambio
# en código se refleje inmediatamente al activar/desactivar la extensión en Kit.
for mod_name in (
    "libs.mavlink.dialect",
    "libs.mavlink.udp_manager",
    "libs.mavlink",
    "navsim_python.time_reference",
    "navsim_python.georeference",
    "navsim_python.drone_identity",
    "navsim_python.drone_config",
    "navsim_python.uav_control",
    "navsim_python.ui_builder",
    "navsim_python.extension",
    "extensions.navsim.navsim_python.time_reference",
    "extensions.navsim.navsim_python.georeference",
    "extensions.navsim.navsim_python.drone_identity",
    "extensions.navsim.navsim_python.drone_config",
    "extensions.navsim.navsim_python.uav_control",
    "extensions.navsim.navsim_python.ui_builder",
    "extensions.navsim.navsim_python.extension",
):
    if mod_name in sys.modules:
        try:
            importlib.reload(sys.modules[mod_name])
        except Exception:
            pass

from .uav_control import UAVControl

try:
    from .extension import *
except ModuleNotFoundError:
    pass
