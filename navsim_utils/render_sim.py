import sys
import traceback
import time
from datetime import timedelta

# 1. Initialize SimulationApp directly using Isaac Sim 6.0 module path
from isaacsim.simulation_app import SimulationApp

SIM_CONFIG = {
    "headless": True,
    "width": 1920,
    "height": 1080,
    "renderer": "RayTracedLighting"
}

simulation_app = SimulationApp(SIM_CONFIG)

# 2. Post-initialization imports
import omni.usd
import omni.kit.app
import omni.timeline
import omni.replicator.core as rep
import omni.kit.viewport.utility as vp_utils


class ProgressTracker:
    """
    Tracks simulation progress, rendering speed, elapsed time, and ETA.
    Uses an Exponential Moving Average (EMA) to smooth out network streaming jitter.
    """
    def __init__(self, start_frame: int, total_frames: int, smoothing: float = 0.85):
        self.start_frame = start_frame
        self.total_frames = total_frames
        self.total_to_render = total_frames - start_frame
        self.smoothing = smoothing
        self.start_time = None
        self.last_frame_time = None
        self.avg_frame_time = None

    def start(self):
        """Initializes the reference timestamps."""
        self.start_time = time.perf_counter()
        self.last_frame_time = self.start_time

    def step(self, current_frame: int) -> str:
        """
        Updates metrics after completing a frame and returns a formatted status string.
        """
        now = time.perf_counter()
        frame_duration = now - self.last_frame_time
        self.last_frame_time = now

        # Compute smoothed frame duration using Exponential Moving Average
        if self.avg_frame_time is None:
            self.avg_frame_time = frame_duration
        else:
            self.avg_frame_time = (self.smoothing * self.avg_frame_time) + ((1.0 - self.smoothing) * frame_duration)

        rendered_count = (current_frame - self.start_frame) + 1
        remaining_frames = max(0, self.total_frames - (current_frame + 1))
        percentage = (rendered_count / self.total_to_render) * 100.0 if self.total_to_render > 0 else 100.0

        elapsed_seconds = int(now - self.start_time)
        eta_seconds = int(remaining_frames * self.avg_frame_time)

        elapsed_str = str(timedelta(seconds=elapsed_seconds))
        eta_str = str(timedelta(seconds=eta_seconds))
        fps = (1.0 / self.avg_frame_time) if self.avg_frame_time > 0 else 0.0

        return (
            f"[PROGRESS] Frame {current_frame + 1}/{self.total_frames} ({percentage:6.2f}%) | "
            f"Speed: {self.avg_frame_time:5.2f}s/frame ({fps:4.2f} FPS) | "
            f"Elapsed: {elapsed_str} | ETA: {eta_str}"
        )


def load_extension(ext_search_path: str, ext_id: str) -> bool:
    """
    Registers a custom directory in the Omniverse extension registry
    and enables the requested extension.
    """
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    ext_manager.add_path(ext_search_path)
    print(f"[INFO] Added custom extension search path: {ext_search_path}")

    # Allow Kit to process the newly registered extension path
    simulation_app.update()

    success = ext_manager.set_extension_enabled_immediate(ext_id, True)
    if success:
        print(f"[SUCCESS] Custom extension '{ext_id}' is now loaded and active.")
    else:
        print(f"[ERROR] Could not load '{ext_id}'. Check path and extension.toml.")
    return success


def setup_cesium_camera_binding(camera_path: str):
    """
    Cesium for Omniverse relies on an active Viewport window to compute tile LOD and culling.
    In headless mode, Replicator render products don't expose a viewport window by default.
    This binds the target camera to an offscreen Viewport API instance.
    """
    viewport_api = vp_utils.get_active_viewport()
    if not viewport_api:
        vp_utils.create_viewport_window("Viewport", camera_path=camera_path)
        viewport_api = vp_utils.get_viewport_from_window_name("Viewport")
        print(f"[CESIUM SETUP] Created offscreen Viewport bound to: {camera_path}")
    else:
        viewport_api.set_active_camera(camera_path)
        print(f"[CESIUM SETUP] Bound active Viewport camera to: {camera_path}")

    return viewport_api


def run_headless_simulation(
    usd_stage_path: str,
    output_directory: str,
    total_frames: int,
    camera_paths: list[str],
    start_frame: int = 0,
    idle_steps_between_frames: int = 5,
    initial_warmup_steps: int = 400,
    log_interval: int = 1
):
    """
    Main simulation loop supporting:
    - Offscreen Viewport creation for Cesium tile LOD calculations.
    - Initial streaming warm-up cycles for tile downloading and decoding.
    - Fast-forward physics without rendering up to start_frame.
    - Idle update steps (with paused physics) between frames for geospatial streaming.
    - Real-time progress and ETA calculation.
    - Safe shutdown and buffer flushing on finish or interruption.
    """
    print(f"[INFO] Opening USD stage: {usd_stage_path}")
    omni.usd.get_context().open_stage(usd_stage_path)

    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("[ERROR] Failed to load the USD stage.")
        simulation_app.close()
        return

    # Verify cameras in the stage
    valid_cameras = [cam for cam in camera_paths if stage.GetPrimAtPath(cam).IsValid()]
    if not valid_cameras:
        print("[ERROR] No valid cameras found in the stage. Aborting.")
        simulation_app.close()
        return

    print(f"[INFO] Validated {len(valid_cameras)}/{len(camera_paths)} cameras.")

    # Bind the main camera to an offscreen viewport so Cesium calculates LOD frustum
    setup_cesium_camera_binding(valid_cameras[0])

    # Initial network & Cesium tile streaming warm-up before any frame capture
    print(f"[INFO] Pumping {initial_warmup_steps} initial engine steps for Cesium tile resolution...")
    for step in range(initial_warmup_steps):
        simulation_app.update()
        if step % 100 == 0:
            print(f" -> Cesium streaming warm-up step {step}/{initial_warmup_steps}")

    timeline = omni.timeline.get_timeline_interface()
    writer = None
    render_products = []
    current_frame = 0

    try:
        # -------------------------------------------------------------
        # PHASE 1: Fast-forward physics without rendering (Warm-Up)
        # -------------------------------------------------------------
        if start_frame > 0:
            print(f"[INFO] Fast-forwarding physics from frame 0 up to {start_frame} (No rendering)...")
            timeline.play()

            ff_start_time = time.perf_counter()
            for f in range(start_frame):
                current_frame = f
                # Pure physics and extension tick (RTX rendering pass is skipped)
                simulation_app.update()

                if f % 100 == 0 or f == start_frame - 1:
                    ff_elapsed = time.perf_counter() - ff_start_time
                    ff_fps = (f + 1) / ff_elapsed if ff_elapsed > 0 else 0.0
                    print(f" -> Fast-forwarded physics: frame {f + 1}/{start_frame} ({ff_fps:.1f} FPS)")

            print(f"[INFO] Fast-forward complete at frame {start_frame}.")

        # -------------------------------------------------------------
        # PHASE 2: Setup Replicator only when rendering starts
        # -------------------------------------------------------------
        print("[INFO] Initializing Replicator render products and writer...")
        for cam_path in valid_cameras:
            rp = rep.create.render_product(cam_path, (SIM_CONFIG["width"], SIM_CONFIG["height"]))
            render_products.append(rp)

        writer = rep.WriterRegistry.get("BasicWriter")
        writer.initialize(output_dir=output_directory, rgb=True)
        writer.attach(render_products)

        with rep.trigger.on_frame():
            pass

        timeline.play()
        print(f"[INFO] Starting rendering loop from frame {start_frame} to {total_frames}...")

        # Initialize progress tracker
        tracker = ProgressTracker(start_frame=start_frame, total_frames=total_frames)
        tracker.start()

        # -------------------------------------------------------------
        # PHASE 3: Capture, idle streaming loop, and progress reporting
        # -------------------------------------------------------------
        for current_frame in range(start_frame, total_frames):

            # Keep camera assigned to the active viewport in case of dynamic stage modifications
            active_vp = vp_utils.get_active_viewport()
            if active_vp:
                active_vp.set_active_camera(valid_cameras[0])

            # Idle steps to let Cesium stream new tiles as camera coordinates change
            if idle_steps_between_frames > 0:
                timeline.pause()
                for _ in range(idle_steps_between_frames):
                    simulation_app.update()
                timeline.play()

            # Advance physics and execute UAV kinematics for this frame
            simulation_app.update()

            # Step Replicator to capture and dispatch render passes to the writer queue
            rep.orchestrator.step(pause_timeline=False)

            # Progress and ETA reporting
            progress_msg = tracker.step(current_frame)
            if (current_frame - start_frame + 1) % log_interval == 0 or current_frame == total_frames - 1:
                print(progress_msg)

    except Exception as e:
        print("\n" + "!" * 70)
        print(f"[CRITICAL FAULT] Simulation interrupted at frame: {current_frame}")
        print(f"[CRITICAL FAULT] Error details: {e}")
        print("!" * 70 + "\n")
        traceback.print_exc()

    finally:
        print("\n[INFO] Shutting down simulation pipeline...")
        timeline.stop()
        rep.orchestrator.stop()

        # Wait for asynchronous disk writer threads to flush all captured images
        if writer is not None:
            print("[INFO] Flushing Replicator I/O queue to disk...")
            rep.orchestrator.wait_until_complete()

        print(f"[SUCCESS] Intact frames through frame {current_frame} are safely written to: {output_directory}")
        simulation_app.close()


# --- Execution Entry Point ---
if __name__ == "__main__":
    # Load required extensions
    load_extension(
        ext_search_path="/home/usuario/navsim/extensions",
        ext_id="uav_control_bridge-0.1.0"
    )
    load_extension(
        ext_search_path="/home/ertete/.local/share/ov/data/Kit/Isaac-Sim Full/6.0/exts/3/Cesium",
        ext_id="cesium.omniverse-0.29.0"
    )
    load_extension(
        ext_search_path="/home/ertete/.local/share/ov/data/Kit/Isaac-Sim Full/6.0/exts/3/Cesium",
        ext_id="cesium.usd.plugins-0.9.0"
    )

    USD_PATH = "/home/ertete/Escritorio/UCLM/TRABAJO/navsim/sims/madrid/main_scene.usd"
    OUTPUT_DIR = "/home/ertete/Escritorio/UCLM/TRABAJO/ISAACSIM/RENDERIZADOS/MADRID"

    CAMERA_PATHS = [
        "/World/UAVs/UAV_0/Camera",
    ]

    TOTAL_FRAMES = 14400
    START_FRAME = 0
    IDLE_STEPS_BETWEEN_FRAMES = 128
    INITIAL_WARMUP_STEPS = 1000
    LOG_INTERVAL = 1

    run_headless_simulation(
        usd_stage_path=USD_PATH,
        output_directory=OUTPUT_DIR,
        total_frames=TOTAL_FRAMES,
        camera_paths=CAMERA_PATHS,
        start_frame=START_FRAME,
        idle_steps_between_frames=IDLE_STEPS_BETWEEN_FRAMES,
        initial_warmup_steps=INITIAL_WARMUP_STEPS,
        log_interval=LOG_INTERVAL
    )