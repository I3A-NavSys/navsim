from pxr import Usd
import sys, os
import omni
import omni.usd
import omni.ext
import carb.events

from uspace.flight_plan.command import Command
import pickle   # Serialization
import base64   # Parsing to string



##############################################################################
# Adding root 'ov' folder to sys.path

current_path = os.path.abspath(os.path.dirname(__file__))
while True:
    if os.path.basename(current_path) == 'ov':
        project_root_path = current_path
        break
    parent_path = os.path.dirname(current_path)
    if parent_path == current_path:
        raise RuntimeError("No se encontró el directorio 'ov' en la ruta.")
    current_path = parent_path
# print(f"Directorio raíz del proyecto: {project_root_path}")

if project_root_path not in sys.path:
    sys.path.append(project_root_path)


##############################################################################
# Accedemos al escenario en el entorno de Isaac Sim

context = omni.usd.get_context()
stage: Usd.Stage = context.get_stage()

# Get the bus event stream
event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()


##############################################################################

num_UAVs = 20
for i in range(num_UAVs):
    UAV_prim_path = f"/Fleet/UAV{i}"

    # Create the event to send commands to the UAV
    UAV_EVENT = carb.events.type_from_string("NavSim." + str(UAV_prim_path))
            
    # Set command data structure
    command = Command(
                    on   = True, 
                    velX = 5, 
                    velY = 0, 
                    velZ = 1,
                    rotZ = 0.2,
                    duration = 10)

    serialized_command = base64.b64encode(pickle.dumps(command)).decode('utf-8')
    event_stream.push(UAV_EVENT, payload={"method": "eventFn_RemoteCommand", "command": serialized_command})
    # print(f"Command sent: {command}")







# from operators.USpaceOperator import USpaceOperator
# from planners.Waypoint import Waypoint
# from planners.FlightPlan import FlightPlan

# # Crear una instancia de UspaceOperator
# op = USpaceOperator("Operador Uno")

# # Acceder al atributo name
# print(op.name)  # Imprime: Operador Uno



# # Ejemplo de uso
# waypoint = Waypoint(label='WP1', t=10, pos=[1, 2, 3], vel=[0.1, 0.2, 0.3])
# print(waypoint.label)
# print(waypoint.t)
# print(waypoint.pos)
# print(waypoint.vel)
# print(waypoint.mandatory)
