# The following modules need to be imported before the app is launched
import argparse
from isaaclab.app import AppLauncher

# Add argparse arguments
parser = argparse.ArgumentParser(
    description="Tutorial on using the interactive scene interface.")
parser.add_argument("--num_envs", type=int, default=2,
                    help="Number of environments to spawn.")

# Append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)

# Parse the arguments
args_cli = parser.parse_args()

# Launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

# Standard library imports
import os

# Related third party imports
import torch
import isaaclab.sim as sim_utils
import isaaclab.envs.mdp as mdp
from isaaclab.envs import ManagerBasedEnv
from isaaclab.envs import ManagerBasedEnvCfg
from isaaclab.actuators import DCMotorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.managers import EventTermCfg
from isaaclab.managers import ObservationTermCfg
from isaaclab.managers import ObservationGroupCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

# Get local resources path
root_navsim_path = os.path.dirname(os.path.abspath(__file__))


@configclass
class QuadcopterSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # Ground plane
    ground = AssetBaseCfg(prim_path="/World/DefaultGroundPlane",
                          spawn=sim_utils.GroundPlaneCfg())

    # Lights
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(
            intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # Quadcopter
    quadcopter: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/quadcopter",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.abspath(os.path.join(
                root_navsim_path, "quadcopter", "quadcopter.usd")),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                rigid_body_enabled=True)
        ),
        actuators={
            "motor_NE": DCMotorCfg(
                joint_names_expr=["motor_NE"],
                stiffness=0.0,
                damping=0.0,
                saturation_effort=100000.0,
                velocity_limit=100000.0,
                effort_limit=100000.0
            ),
            "motor_NW": DCMotorCfg(
                joint_names_expr=["motor_NW"],
                stiffness=0.0,
                damping=0.0,
                saturation_effort=100000.0,
                velocity_limit=100000.0,
                effort_limit=100000.0
            ),
            "motor_SE": DCMotorCfg(
                joint_names_expr=["motor_SE"],
                stiffness=0.0,
                damping=0.0,
                saturation_effort=100000.0,
                velocity_limit=100000.0,
                effort_limit=100000.0
            ),
            "motor_SW": DCMotorCfg(
                joint_names_expr=["motor_SW"],
                stiffness=0.0,
                damping=0.0,
                saturation_effort=100000.0,
                velocity_limit=100000.0,
                effort_limit=100000.0
            )
        }
    )


@configclass
class ActionsCfg:
    """Action specification for the enviroment"""
    motor_speeds = mdp.JointEffortActionCfg(
        asset_name="quadcopter",
        joint_names=["motor_NE", "motor_NW", "motor_SE", "motor_SW"])


@configclass
class ObservationsCfg:
    """Observation specification for the enviroment"""

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observations for policy group"""
        joint_pos_rel = ObservationTermCfg(
            func=mdp.joint_pos_rel,
            params={"asset_cfg": SceneEntityCfg("quadcopter")})
        joint_vel_rel = ObservationTermCfg(
            func=mdp.joint_vel_rel,
            params={"asset_cfg": SceneEntityCfg("quadcopter")})

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # Observation group
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg():
    """Configuration for events"""
    # On startup
    add_quadcopter_mass = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg(name="quadcopter"),
            "pose_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.5, 0.5),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (-3.14, 3.14)
            },
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0)
            }
        }
    )


@configclass
class QuadcopterEnvCfg(ManagerBasedEnvCfg):
    """Configuration for the quadcopter enviroment."""
    # Scene settings
    scene = QuadcopterSceneCfg(num_envs=64, env_spacing=2.5)
    # Basic settings
    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventCfg()

    def __post_init__(self):
        """Post initialization."""
        # Viewer settings
        self.viewer.eye = [4.5, 0.0, 6.0]
        self.viewer.lookat = [0.0, 0.0, 2.0]
        # Step settings
        self.decimation = 4
        # Simulation settings
        self.sim.dt = 0.005


def main():
    """Main function."""
    # Parse the arguments
    env_cfg = QuadcopterEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.decive = args_cli.device
    # Setup base enviroment
    env = ManagerBasedEnv(cfg=env_cfg)

    # Simulate physics
    count = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            # Reset
            if count % 300 == 0:
                count = 0
                env.reset()
                print("_" * 80)
                print("[INFO]: Resetting environment...")

            # Set all joints to zero
            joint_efforts = torch.zeros_like(env.action_manager.action)

            # Step the environment
            obs, _ = env.step(joint_efforts)

            # Update counter
            count += 1


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
