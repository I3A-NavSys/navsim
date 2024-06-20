import sys
import os

# Añadir el directorio principal al sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

if os.name == 'nt':
    os.system('cls')
else:
    os.system('clear')

from operators.USpaceOperator import USpaceOperator
from planners.FlightPlan import Waypoint

# Crear una instancia de UspaceOperator
op = USpaceOperator("Operador Uno")

# Acceder al atributo name
print(op.name)  # Imprime: Operador Uno








# Ejemplo de uso
waypoint = Waypoint(label='WP1', t=10, pos=[1, 2, 3], vel=[0.1, 0.2, 0.3])
print(waypoint.label)
print(waypoint.t)
print(waypoint.pos)
print(waypoint.vel)
print(waypoint.mandatory)
