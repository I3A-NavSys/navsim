import numpy as np
import random

import omni.kit.window.stage
import omni.usd
from pxr import UsdGeom, Gf
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
import omni.kit.window.file.save_stage_ui
import omni.kit.window.filepicker

def define_scene(project_root_path, sphere_amount, distance, x_level, y_level, offset, amount_vertiports, amount_uavs):
    vertiports_prim_path = "/World/Vertiports"
    uavs_prim_path = "/World/UAVs"
    vertiport_usd_path = project_root_path + "/assets/vertiports/vertiport_aerotaxi.usd"
    aerotaxi_usd_path = project_root_path + "/fleet/UAM_aerotaxi/UAM_aerotaxi.usd"
    environment_scope_path = "/World/Environment"
    light_prim_path = environment_scope_path + "/Light"
    physics_scene_prim_path = environment_scope_path + "/PhysicsScene"

    stage = omni.usd.get_context().get_stage()

    # Ensure the stage is valid
    if not stage:
        print("Failed to get the stage. Ensure the stage is open in Omniverse.")
        return

    # Create a root Xform for organization
    stage.DefinePrim("/World", "Xform")

    # Add grid to stage
    sphere_grid_path = "/World/SphereGrid"
    sphere_grid_prim = stage.DefinePrim(sphere_grid_path, "Xform")
    UsdGeom.XformCommonAPI(sphere_grid_prim).SetTranslate((0, 0, 0))

    x_color = Gf.Vec3f(1.0, 0.0, 0.0)
    y_color = Gf.Vec3f(1.0, 0.843, 0.0)

    sphere_counter = 0
    for i in range(int(-sphere_amount/2), int(sphere_amount/2 + 2)):
        for j in range(int(-sphere_amount/2), int(sphere_amount/2 + 2)):
            sphere_counter += 1

            # Calculate the position for the current node
            x = i * distance + offset
            y = j * distance
            
            # Create the node
            sphere_name = f"Sphere_X_{sphere_counter}"
            create_sphere(stage, sphere_name, (x, y, x_level), sphere_grid_path, x_color)
            sphere_name = f"Sphere_Y_{sphere_counter}"
            create_sphere(stage, sphere_name, (x-offset, y+offset, y_level), sphere_grid_path, y_color)

    # Add vertiports to stage
    stage.DefinePrim(vertiports_prim_path, "Xform")

    current_amount_vertiports = 0
    possible_x = np.arange(-sphere_amount/2, sphere_amount/2 + 2) * distance + offset
    possible_y = np.arange(-sphere_amount/2, sphere_amount/2 + 2) * distance
    registered_vertiport_locs = {}

    while current_amount_vertiports < amount_vertiports:
        x = random.choice(possible_x)
        y = random.choice(possible_y)

        while (x, y) in registered_vertiport_locs:
            x = random.choice(possible_x)
            y = random.choice(possible_y)

        registered_vertiport_locs[(x, y)] = True

        add_prim_reference(f"Vertiport_{current_amount_vertiports}", (x, y, 0), vertiport_usd_path, vertiports_prim_path)
        prim = stage.GetPrimAtPath(f"{vertiports_prim_path}/Vertiport_{current_amount_vertiports}")
        prim.GetAttribute("NavSim:id").Set(f"v_{current_amount_vertiports}")
        current_amount_vertiports += 1

    # Add uavs to stage
    stage.DefinePrim(uavs_prim_path, "Xform")

    vertiport_locs = list(registered_vertiport_locs.keys())
    registered_uav_locs = {}
    current_amount_uavs = 0

    while current_amount_uavs < amount_uavs:
        loc = random.choice(vertiport_locs)

        if loc in registered_uav_locs:
            continue

        registered_uav_locs[loc] = True

        add_prim_reference(f"UAV_{current_amount_uavs}", (loc[0], loc[1], 1.75), aerotaxi_usd_path, uavs_prim_path)
        current_amount_uavs += 1

    # Add light and physics scene to stage
    stage.DefinePrim(environment_scope_path, "Scope")
    light_prim = stage.DefinePrim(light_prim_path, "DomeLight")
    physics_scene_prim = stage.DefinePrim(physics_scene_prim_path, "PhysicsScene")

    light_prim.GetAttribute("inputs:intensity").Set(1000)

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

def add_prim_reference(name, position, usd_path, prim_path):
    # Define the vertiport's prim path
    prim = add_reference_to_stage(usd_path=usd_path, prim_path=f"{prim_path}/{name}")
    
    # Set the vertiport's position
    pos_atr = prim.GetAttribute("xformOp:translate")
    pos_atr.Set(position)
    

def build_scene(project_root_path, sphere_amount, distance, x_level, y_level, offset, amount_vertiports, amount_uavs):
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
        define_scene(project_root_path, sphere_amount, distance, x_level, y_level, offset, amount_vertiports, amount_uavs)

    def save_scene(selected_layers, comment):
        filepicker.show()

    def dont_save_scene(arg1):
        usd_context.new_stage()
        define_scene(project_root_path, sphere_amount, distance, x_level, y_level, offset, amount_vertiports, amount_uavs)

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
        define_scene(project_root_path, sphere_amount, distance, x_level, y_level, offset, amount_vertiports)