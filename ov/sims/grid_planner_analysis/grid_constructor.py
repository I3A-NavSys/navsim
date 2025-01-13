# Import required modules
import omni.usd
from pxr import Usd, UsdGeom, Gf
from pathlib import Path
from omni.isaac.core.utils.stage import get_current_stage, add_reference_to_stage

file_path = Path(__file__).resolve()

# Parameters for the grid
square_size = 10  # Number of spheres per row/column
distance = 100.0   # Distance between sphere centers
x_level = 60
y_level = 100
offset = -50

# Parameters for the vertiports
takeoff_vertiport_loc = (-5*distance, 0, 0)
landing_vertiport_loc = (5*distance, 0, 0)
vertiports_prim_path = "/World/Vertiports"
vertiport_usd_path = file_path.parent.parent.parent / "assets" / "vertiports" / "vertiport_aerotaxi.usd"
vertiport_usd_path = vertiport_usd_path.as_posix()

# Parameters for the aerotaxi
aerotaxi_loc = (-5*distance, 0, 1.75)
aerotaxi_usd_path = file_path.parent.parent.parent / "fleet" / "UAM_aerotaxi" / "UAM_aerotaxi.usd"
aerotaxi_usd_path = aerotaxi_usd_path.as_posix()

# Create a new USD stage
stage = omni.usd.get_context().get_stage()

# Ensure the stage is valid
if not stage:
    print("Failed to get the stage. Ensure the stage is open in Omniverse.")
    exit()

# Create a root Xform for organization
root_path = "/World/SphereGrid"
root_prim = stage.DefinePrim(root_path, "Xform")
UsdGeom.XformCommonAPI(root_prim).SetTranslate((0, 0, 0))

# Function to create a sphere at a specific position
def create_sphere(stage, name, position, parent_path):
    # Define the sphere's prim path
    sphere_path = f"{parent_path}/{name}"
    sphere_prim = stage.DefinePrim(sphere_path, "Sphere")
    
    # Set the sphere's position
    sphere_xform_api = UsdGeom.XformCommonAPI(sphere_prim)
    sphere_xform_api.SetTranslate(position)
    # sphere_xform_api.SetScale((0.5, 0.5, 0.5))

sphere_counter = 0
# Generate the spheres in a grid
for i in range(int(-square_size/2), int(square_size/2 + 2)):
    for j in range(int(-square_size/2), int(square_size/2 + 2)):
        sphere_counter += 1

        # Calculate the position for the current sphere
        x = i * distance + offset
        y = j * distance
        z = x_level  # All spheres are on the same plane
        
        # Create the sphere
        sphere_name = f"Sphere_{sphere_counter}"
        create_sphere(stage, sphere_name, (x, y, z), root_path)

stage.DefinePrim(vertiports_prim_path, "Xform")
vertiport_takeoff_prim = add_reference_to_stage(usd_path=vertiport_usd_path, prim_path=f"{vertiports_prim_path}/vertiport_takeoff")
vertiport_landing_prim = add_reference_to_stage(usd_path=vertiport_usd_path, prim_path=f"{vertiports_prim_path}/vertiport_landing")
aerotaxi_prim = add_reference_to_stage(usd_path=aerotaxi_usd_path, prim_path="/World/aerotaxi")

vertiport_takeoff_prim.GetAttribute("xformOp:translate").Set(takeoff_vertiport_loc)
vertiport_landing_prim.GetAttribute("xformOp:translate").Set(landing_vertiport_loc)
aerotaxi_prim.GetAttribute("xformOp:translate").Set(aerotaxi_loc)

print("Spheres have been created and distributed along the square grid.")