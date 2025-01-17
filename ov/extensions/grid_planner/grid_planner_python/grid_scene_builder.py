import omni.kit.window.stage
import omni.usd
from pxr import UsdGeom, Gf
from omni.isaac.core.utils.stage import add_reference_to_stage, get_current_stage
import omni.kit.window.file.save_stage_ui
import omni.kit.window.filepicker

def define_scene(project_root_path, sphere_amount, distance, x_level, y_level, offset, amount_vertiports):
    vertiports_prim_path = "/World/Vertiports"
    vertiport_usd_path = project_root_path + "/assets/vertiports/vertiport_aerotaxi.usd"
    aerotaxi_usd_path = project_root_path + "/fleet/UAM_aerotaxi/UAM_aerotaxi.usd"
    environment_scope_path = "/World/Environment"
    light_prim_path = "/World/Environment/Light"
    physics_scene_prim_path = "/World/Environment/PhysicsScene"
    time_steps_per_second = 50
    end_time_code = time_steps_per_second * 50

    stage = omni.usd.get_context().get_stage()

    # Ensure the stage is valid
    if not stage:
        print("Failed to get the stage. Ensure the stage is open in Omniverse.")
        return

    # Create a root Xform for organization
    stage.DefinePrim("/World", "Xform")
    stage.DefinePrim(environment_scope_path, "Scope")
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
    light_prim = stage.DefinePrim(light_prim_path, "DomeLight")
    physics_scene_prim = stage.DefinePrim(physics_scene_prim_path, "PhysicsScene")

    stage = get_current_stage()

    light_prim.GetAttribute("inputs:intensity").Set(1000)
    # physics_scene_prim.GetAttribute("physxScene:timeStepsPerSecond").Set(time_steps_per_second)
    stage.GetRootLayer().default_time_codes_per_second = time_steps_per_second
    stage.GetRootLayer().default_end_time_code = end_time_code

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

def build_scene(project_root_path, sphere_amount, distance, x_level, y_level, offset, amount_vertiports):
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
        define_scene(project_root_path, sphere_amount, distance, x_level, y_level, offset, amount_vertiports)

    def save_scene(selected_layers, comment):
        filepicker.show()

    def dont_save_scene(arg1):
        usd_context.new_stage()
        define_scene(project_root_path, sphere_amount, distance, x_level, y_level, offset, amount_vertiports)

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