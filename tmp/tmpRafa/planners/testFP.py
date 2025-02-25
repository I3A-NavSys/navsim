import sys
import os

#borrar consola
if os.name == 'nt':
    os.system('cls')
else:
    os.system('clear')

# Añadir el directorio principal al sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

##############################################################################



from planners.Waypoint import Waypoint
from planners.FlightPlan import FlightPlan


# Crear waypoints
waypoint1 = Waypoint(label='WP1', t= 0,  pos=[  0,   0, 0], vel=[10,  1, 0])
waypoint2 = Waypoint(label='WP2', t=10,  pos=[100, 100, 0], vel=[ 0, 10, 0])





# Crear plan de vuelo
fp = FlightPlan([waypoint1, waypoint2])
fp.SetJLS()

# Mostrar la posición 3D del plan de vuelo
fp.Print()
fp.PositionFigure('Flight Path 3D', 0.1)

