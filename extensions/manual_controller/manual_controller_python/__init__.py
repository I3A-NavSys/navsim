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
    "mavlink.dialect",
    "mavlink.udp_manager",
    "mavlink",
    "manual_controller_python.controller_logic",
    "manual_controller_python.keyboard_input",
    "manual_controller_python.joystick_input",
    "manual_controller_python.extension",
    "extensions.manual_controller.manual_controller_python.controller_logic",
    "extensions.manual_controller.manual_controller_python.keyboard_input",
    "extensions.manual_controller.manual_controller_python.joystick_input",
    "extensions.manual_controller.manual_controller_python.extension",
):
    if mod_name in sys.modules:
        try:
            importlib.reload(sys.modules[mod_name])
        except Exception:
            pass

from .extension import *
