import sys
import os

# Victor imports for the parallel work
from multiprocessing import Process
import time

# ####################################################################################################
# By Victor
# START - Function that is executed in parallel to the graph

def count():
    for i in range(20):
        print('\n',i)

        time.sleep(1)

# END - Function that is executed in parallel to the graph
# ####################################################################################################

#borrar consola
if os.name == 'nt':
    os.system('cls')
else:
    os.system('clear')

# Añadir el directorio principal al sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

##############################################################################



from Waypoint import Waypoint
from FlightPlan import FlightPlan


# Crear waypoints
waypoint1 = Waypoint(label='WP1', t= 0,  pos=[  0,   0, 0], vel=[10,  1, 0])
waypoint2 = Waypoint(label='WP2', t=10,  pos=[100, 100, 0], vel=[ 0, 10, 0])

# Crear plan de vuelo
fp = FlightPlan([waypoint1, waypoint2])
fp.SetJLS()

# Mostrar la posición 3D del plan de vuelo
fp.Print()
# fp.PositionFigure('Flight Path 3D', 0.1)


# ####################################################################################################
# By Victor
# START - Main
if __name__ == '__main__':
    # Create needed processes
    graphP = Process(target=fp.PositionFigure, args=('Flight Path 3D', 0.1))

    # Start those processes
    graphP.start()

    # Run parallel work
    count()
    print('\nEnd of parent process')

    graphP.join()
    print('\nEnd of graph process')

# END - Main
# ####################################################################################################