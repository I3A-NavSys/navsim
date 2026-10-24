"""
Librerías compartidas de NavSim (libs):
- mavlink: Protocolo de comunicaciones MAVLink v2 y gestor UDP.
- flight_plan: Modelado y cinemática de planes de vuelo y waypoints.
"""

import sys
from pathlib import Path

# Garantizar que 'libs' y sus submódulos sean accesibles
_LIBS_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _LIBS_DIR.parent

if str(_REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT))

# Alias para compatibilidad hacia atrás si algún script antiguo importa 'mavlink' directamente
try:
    import libs.mavlink as _mavlink
    sys.modules.setdefault("mavlink", _mavlink)
    sys.modules.setdefault("mavlink.dialect", _mavlink.dialect)
    sys.modules.setdefault("mavlink.udp_manager", _mavlink.udp_manager)
except Exception:
    pass

try:
    import libs.flight_plan as _flight_plan
    sys.modules.setdefault("flight_plan", _flight_plan)
except Exception:
    pass
