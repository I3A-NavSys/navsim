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


# Crear algunos waypoints de prueba
wp1 = Waypoint(label='WP1', t=0, pos=[0, 0, 0], vel=[1, 1, 0])
wp2 = Waypoint(label='WP2', t=2, pos=[2, 2, 0], vel=[1, 1, 0])
wp3 = Waypoint(label='WP3', t=4, pos=[4, 4, 0], vel=[1, 1, 0])

# Crear un plan de vuelo con los waypoints
fp = FlightPlan([wp1, wp2])

# Añadir un waypoint al final del plan de vuelo
fp.AppendWaypoint(wp3)

# Imprimir el plan de vuelo
print("Flight Plan Waypoints:")
fp.Print()

fp.RemoveWaypointAtTime(0)
print("Flight Plan Waypoints:")
fp.Print()

print(fp)