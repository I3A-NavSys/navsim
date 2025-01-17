import omni.kit.window.stage
import omni.usd
from pxr import UsdGeom, Gf
from omni.isaac.core.utils.stage import add_reference_to_stage
import omni.kit.window.file.save_stage_ui
import omni.kit.window.filepicker

def define_scene(project_root_path, sphere_amount, distance, x_level, y_level, offset, prim_locations):
    vertiports_prim_path = "/World/Vertiports"
    vertiport_usd_path = project_root_path + "/assets/vertiports/vertiport_aerotaxi.usd"
    aerotaxi_usd_path = project_root_path + "/fleet/UAM_aerotaxi/UAM_aerotaxi.usd"

    vertiport = prim_locations["Init vertiport"]
    init_vertiport_loc = (vertiport[0].model.get_value_as_float(), vertiport[1].model.get_value_as_float(), 
                            vertiport[2].model.get_value_as_float())
    
    vertiport = prim_locations["End vertiport"]
    end_vertiport_loc = (vertiport[0].model.get_value_as_float(), vertiport[1].model.get_value_as_float(), 
                            vertiport[2].model.get_value_as_float())
    
    aerotaxi_loc = (init_vertiport_loc[0], init_vertiport_loc[1], init_vertiport_loc[2] + 1.75)

    stage = omni.usd.get_context().get_stage()

    # Ensure the stage is valid
    if not stage:
        print("Failed to get the stage. Ensure the stage is open in Omniverse.")
        return

    # Create a root Xform for organization
    sphere_grid_prim = stage.DefinePrim("/World", "Xform")
    sphere_grid_path = "/World/SphereGrid"
    sphere_grid_prim = stage.DefinePrim(sphere_grid_path, "Xform")
    UsdGeom.XformCommonAPI(sphere_grid_prim).SetTranslate((0, 0, 0))

    x_color = Gf.Vec3f(1.0, 0.0, 0.0)
    y_color = Gf.Vec3f(1.0, 0.843, 0.0)
    sphere_counter = 0
    # Generate the spheres in a grid
    for i in range(int(-sphere_amount/2), int(sphere_amount/2 + 2)):
        for j in range(int(-sphere_amount/2), int(sphere_amount/2 + 2)):
            sphere_counter += 1

            # Calculate the position for the current sphere
            x = i * distance + offset
            y = j * distance
            
            # Create the sphere
            sphere_name = f"Sphere_X_{sphere_counter}"
            create_sphere(stage, sphere_name, (x, y, x_level), sphere_grid_path, x_color)
            sphere_name = f"Sphere_Y_{sphere_counter}"
            create_sphere(stage, sphere_name, (x-offset, y+offset, y_level), sphere_grid_path, y_color)

    stage.DefinePrim(vertiports_prim_path, "Xform")
    init_vertiport_prim = add_reference_to_stage(usd_path=vertiport_usd_path, prim_path=f"{vertiports_prim_path}/vertiport_takeoff")
    end_vertiport_prim = add_reference_to_stage(usd_path=vertiport_usd_path, prim_path=f"{vertiports_prim_path}/vertiport_landing")
    aerotaxi_prim = add_reference_to_stage(usd_path=aerotaxi_usd_path, prim_path="/World/aerotaxi")

    init_vertiport_prim.GetAttribute("xformOp:translate").Set(init_vertiport_loc)
    end_vertiport_prim.GetAttribute("xformOp:translate").Set(end_vertiport_loc)
    aerotaxi_prim.GetAttribute("xformOp:translate").Set(aerotaxi_loc)

def create_sphere(stage, name, position, parent_path, color):
    # Define the sphere's prim path
    sphere_path = f"{parent_path}/{name}"
    sphere_prim = stage.DefinePrim(sphere_path, "Sphere")
    
    # Set the sphere's position
    sphere_xform_api = UsdGeom.XformCommonAPI(sphere_prim)
    sphere_xform_api.SetTranslate(position)
    # sphere_xform_api.SetScale((0.5, 0.5, 0.5))

    geom = UsdGeom.Gprim(sphere_prim)
    geom.GetDisplayColorAttr().Set([color])

def build_scene(project_root_path, sphere_amount, distance, x_level, y_level, offset, prim_locations):
    """Check for changes in the scene, prompt the user to save if necessary, and create a new scene."""
    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()
    root_layer = stage.GetRootLayer()

    def on_save_completed(filename, selected_path):
        if not filename.endswith(".usd"):
            filename += ".usd"
        
        target_dir = selected_path + filename
        usd_context.save_as_stage(target_dir)
        filepicker.hide()
        usd_context.new_stage()
        define_scene(project_root_path, sphere_amount, distance, x_level, y_level, offset, prim_locations)

    def save_scene(selected_layers, comment):
        filepicker.show()

    def dont_save_scene(arg1):
        usd_context.new_stage()
        define_scene(project_root_path, sphere_amount, distance, x_level, y_level, offset, prim_locations)

    dialog = omni.kit.window.file.StageSaveDialog(
        on_save_fn=save_scene,
        on_dont_save_fn=dont_save_scene,
        enable_dont_save=True
    )

    filepicker = omni.kit.window.filepicker.FilePickerDialog(
        title="Save Scene",
        click_apply_handler=on_save_completed,
        apply_button_label="Save"
    )
    filepicker.hide()

    # Check for unsaved changes and prompt the user if necessary
    if root_layer.dirty:
        dialog.show()
    else:
        usd_context.new_stage()
        define_scene(project_root_path, sphere_amount, distance, x_level, y_level, offset, prim_locations)