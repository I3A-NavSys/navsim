##############################################################################
#borrar consola
import os
if os.name == 'nt':
    os.system('cls')
else:
    os.system('clear')
##############################################################################
# Adding root 'project' folder to sys.path
import sys, os
project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)
# # Verificar que la ruta se ha añadido correctamente
# print("Rutas en sys.path:")
# for path in sys.path:
#     print(path)
##############################################################################

from uspace.flight_plan.waypoint import Waypoint
from uspace.flight_plan.flight_plan import FlightPlan

# Crear waypoints
waypoint1 = Waypoint(label='WP1', t= 5,  pos=[  0,   0, 0], vel=[10,  1, 0])
waypoint2 = Waypoint(label='WP2', t=10,  pos=[100, 100, 0], vel=[ 0, 10, 0])

# Crear plan de vuelo
fp = FlightPlan([waypoint1, waypoint2])
fp.SetJLS()

# Mostrar la posición 3D del plan de vuelo
fp.Print()
fp.PositionFigure("FP1: POSITION", 0.1)
fp.VelocityFigure("FP1: VELOCITY", 0.1)

for t in range(0, 11):
    i = fp.GetIndexFromTime(t)
    print(f"Index at time {t}: {i}")