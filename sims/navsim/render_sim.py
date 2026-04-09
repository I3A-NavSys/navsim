# 1. Initialize the SimulationApp FIRST
from omni.isaac.kit import SimulationApp

sim_config = {
    "headless": True,
    "width": 1920,
    "height": 1080,
    "renderer": "RayTracedLighting"
}

simulation_app = SimulationApp(sim_config)

# 2. Import Omniverse modules AFTER initialization
import omni.usd
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import traceback  # Required to print the exact error trace from your extension

def load_custom_extension(ext_search_path: str, ext_id: str):
    """
    Registers a custom directory in the Omniverse extension registry 
    and forcefully enables the specified extension.
    """
    # CRITICAL FIX: Retrieve the extension manager through the core application instance
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    
    ext_manager.add_path(ext_search_path)
    print(f"[INFO] Added custom extension search path: {ext_search_path}")
    
    simulation_app.update()
    
    success = ext_manager.set_extension_enabled_immediate(ext_id, True)
    if success:
        print(f"[SUCCESS] Custom extension '{ext_id}' is now loaded and active.")
    else:
        print(f"[ERROR] Could not load '{ext_id}'. Verify the path and the extension.toml file.")

def run_headless_simulation(usd_stage_path: str, output_directory: str, total_frames: int, camera_paths: list[str]):
    """
    Main loop: Loads the scene, setups render, and steps physics/rendering with fault tolerance.
    """
    print(f"[INFO] Loading stage: {usd_stage_path}")
    omni.usd.get_context().open_stage(usd_stage_path)
    
    for _ in range(10):
        simulation_app.update()

    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("[ERROR] Failed to load the USD stage.")
        simulation_app.close()
        return

    # Set up Replicator
    print("[INFO] Configuring Omniverse Replicator...")
    render_products = []
    
    for cam_path in camera_paths:
        if stage.GetPrimAtPath(cam_path).IsValid():
            # Create a render product for each valid camera
            rp = rep.create.render_product(cam_path, (sim_config["width"], sim_config["height"]))
            render_products.append(rp)
        else:
            print(f"[WARN] Camera prim not found: {cam_path}")

    if not render_products:
        print("[ERROR] No cameras available for rendering. Exiting.")
        simulation_app.close()
        return
    
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=output_directory, rgb=True)
    writer.attach(render_products)

    with rep.trigger.on_frame():
        pass

    timeline = omni.timeline.get_timeline_interface()
    timeline.play()

    print(f"[INFO] Starting physics and rendering loop...")

    # We initialize the frame counter outside the loop so the 'except' block can read its final value
    current_frame = 0

    try:
        # Execution Loop
        for current_frame in range(total_frames):
            
            # This call advances physics, triggers your custom extension, and renders the frame
            simulation_app.update()
            rep.orchestrator.step()
            
            if current_frame % 10 == 0:
                print(f" -> Processed and rendered frame {current_frame}/{total_frames}")

    except Exception as e:
        # This block catches any error thrown by your custom extension or the physics engine
        print("\n" + "!" * 70)
        print(f"[CRITICAL FAULT] The simulation crashed exactly at frame: {current_frame}")
        print(f"[CRITICAL FAULT] Exception message: {str(e)}")
        print("!" * 70 + "\n")
        
        # Print the full Python stack trace so you can debug your extension's code
        traceback.print_exc()
        
        print("\n[INFO] Commencing emergency data rescue. Forcing Replicator to flush the I/O queue...")

    finally:
        # The 'finally' block ALWAYS executes, regardless of success or failure.
        # It guarantees that your rendered frames are safely committed to the disk.
        print("[INFO] Halting the physics timeline...")
        timeline.stop()
        rep.orchestrator.stop()
        
        # CRITICAL: Wait for all background image writing threads to finish
        print("[INFO] Waiting for asynchronous disk writes to complete...")
        rep.orchestrator.wait_until_complete()
        
        print(f"[SUCCESS] All intact frames up to frame {current_frame} have been saved to: {output_directory}")
        print("[INFO] Closing Isaac Sim safely to release GPU memory...")
        simulation_app.close()

# --- Execution Entry Point ---
if __name__ == "__main__":
    EXTENSIONS_FOLDER_PATH = "/home/usuario/navsim/extensions"
    EXTENSION_ID = "navsim-0.1.0"
    USD_PATH = "/home/usuario/navsim/sims/navsim/main_scene.usd"
    OUTPUT_DIR = "/home/usuario/navsim/sims/navsim/render"
    CAMERA_PATHS = [
        "/World/UAV_operators/UAV_OP_01/Fleet/UAV_03/UAV_03_CAM",
        "/World/UAV_operators/UAV_OP_01/Fleet/UAV_04/UAV_04_CAM",
        "/VERT_OP_01_CAM",
        "/VERT_OP_02_CAM",
        "/VERT_OP_03_CAM",
        "/VERT_OP_04_CAM",
        "/VERT_OP_07_CAM",
        "/VERT_OP_08_CAM",
        "/VERT_OP_09_CAM",
        "/VERT_OP_10_CAM",
        "/GRID_CAM",
    ]
    TOTAL_FRAMES = 150000
    
    load_custom_extension(EXTENSIONS_FOLDER_PATH, EXTENSION_ID)
    
    run_headless_simulation(
        usd_stage_path=USD_PATH,
        output_directory=OUTPUT_DIR,
        total_frames=TOTAL_FRAMES,
        camera_paths=CAMERA_PATHS
    )