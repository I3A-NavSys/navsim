##############################################################################
#borrar consola
import os
if os.name == 'nt':
    os.system('cls')
else:
    os.system('clear')

##############################################################################
# Añadir el directorio principal al sys.path
import sys, os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))

##############################################################################

from uspace.flightplan.Waypoint   import Waypoint
from uspace.flightplan.FlightPlan import FlightPlan


# Crear waypoints
wp1L = Waypoint(label="wp1L", pos=[-190, -119, 48.1])
wp1P = Waypoint(label="wp1P", pos=[-190, -119, 48.1])
wp1 = Waypoint(label="wp1", pos=[-190, -119, 70])
wp2 = Waypoint(label="wp2", pos=[-90, -19, 70])
wp3 = Waypoint(label="wp3", pos=[-90, 181, 70])
wp4 = Waypoint(label="wp4", pos=[110, 181, 70])
wp5 = Waypoint(label="wp5", pos=[110, -119, 70])
wp6L = Waypoint(label="wp6L", pos=[-152, -106, 49.1])
wp6P = Waypoint(label="wp6P", pos=[-152, -106, 49.1])
wp6 = Waypoint(label="wp6", pos=[-152, -106, 70])

fp = FlightPlan()
fp.radius = 2
fp.AppendWaypoint(wp1L)
wp1P.t = fp.FinishTime() + 5
fp.SetWaypoint(wp1P)
fp.AppendWaypoint(wp1)
fp.AppendWaypoint(wp2)
fp.AppendWaypoint(wp3)
fp.AppendWaypoint(wp4)
fp.AppendWaypoint(wp5)
fp.AppendWaypoint(wp6)
fp.AppendWaypoint(wp6L)
wp6P.t = fp.FinishTime() + 5
fp.SetWaypoint(wp6P)

fp.SetTimeFromVel("wp1", 2)
fp.SetTimeFromVel("wp2", 8)
fp.SetTimeFromVel("wp3", 8)
fp.SetTimeFromVel("wp4", 8)
fp.SetTimeFromVel("wp5", 8)
fp.SetTimeFromVel("wp6", 8)
fp.SetTimeFromVel("wp6L", 2)

# wp3.SetV0000(wp4)

fp.SetV0000()
fp.PositionFigure("FP1: POSITION", 0.01)
fp.VelocityFigure("FP1: VELOCITY", 0.01)

# waypoint1 = Waypoint(label='WP1', t= 5,  pos=[  0,   0, 0], vel=[10,  1, 0])
# waypoint2 = Waypoint(label='WP2', t=10,  pos=[100, 100, 0], vel=[ 0, 10, 0])

# Crear plan de vuelo
# fp = FlightPlan([waypoint1, waypoint2])
# fp.SetJLS()

# Mostrar la posición 3D del plan de vuelo
# fp.Print()
# fp.PositionFigure('Flight Path 3D', 0.1)

# for t in range(0, 11):
#     i = fp.GetIndexFromTime(t)
#     print(f"Index at time {t}: {i}")


