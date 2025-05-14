# Standard library imports
import os

# Related third party imports
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.envs import mdp
from isaaclab.actuators import DCMotorCfg
from isaaclab.managers import EventTermCfg
from isaaclab.managers import ObservationGroupCfg
from isaaclab.managers import ObservationTermCfg
from isaaclab.managers import RewardTermCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

# User specific imports
from .mdp.actions.actions_cfg import QuadcopterMotorActionCfg
from .mdp import rewards

# Get local resources path
root_isaac_lab_path = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..'))


##
# Scene definition
##


@configclass
class QuadcopterSceneCfg(InteractiveSceneCfg):
    """Scene configuration."""

    # Ground plane
    ground = AssetBaseCfg(prim_path="/World/DefaultGroundPlane",
                          spawn=sim_utils.GroundPlaneCfg())

    # Lights
    dome_light = AssetBaseCfg(prim_path="/World/DomeLight",
                              spawn=sim_utils.DomeLightCfg(
                                  intensity=3000.0, color=(0.75, 0.75, 0.75))
                              )

    # Quadcopter
    quadcopter: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/quadcopter",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.abspath(os.path.join(root_isaac_lab_path,
                                                  "assets",
                                                  "quadcopter",
                                                  "quadcopter.usd")),
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


##
# MDP settings
##


@configclass
class ActionsCfg:
    """Action specification for the enviroment"""
    motor_speeds = QuadcopterMotorActionCfg(asset_name="quadcopter",
                                            joint_names=[
                                                "motor_NE", "motor_NW", 
                                                "motor_SE", "motor_SW"],
                                            scale=0.5)


@configclass
class ObservationsCfg:
    """Observation specification for the enviroment"""

    @configclass
    class PolicyCfg(ObservationGroupCfg):
        """Observations for policy group"""
        base_lin_vel = ObservationTermCfg(
            func=mdp.base_lin_vel,
            params={"asset_cfg": SceneEntityCfg("quadcopter")})
        base_ang_vel = ObservationTermCfg(
            func=mdp.base_ang_vel,
            params={"asset_cfg": SceneEntityCfg("quadcopter")})

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # Observation group
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg():
    """Configuration for events"""
    # On reset
    reset_quadcopter_position = EventTermCfg(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg(name="quadcopter"),
            "pose_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (2.5, 2.5),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0)
            },
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0)
            }
        }
    )


@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # (1) Constant running reward
    alive = RewardTermCfg(func=mdp.is_alive, weight=1.0)
    # (2) Failure penalty
    terminating = RewardTermCfg(func=mdp.is_terminated, weight=-3.0)
    # (3) Primary task: keep linear velocity close to zero
    quadcopter_lin_vel = RewardTermCfg(
        func=rewards.lin_vel_diff,
        weight=1.0
    )
    # (4) Primary task: keep angular velocity close to zero
    quadcopter_ang_vel = RewardTermCfg(
        func=rewards.ang_vel_diff,
        weight=1.0
    )


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = TerminationTermCfg(func=mdp.time_out, time_out=True)
    # (2) Quadcopter too close to ground
    quadcopter_height = TerminationTermCfg(
        func=mdp.root_height_below_minimum,
        params={"minimum_height": 0.25,
                "asset_cfg": SceneEntityCfg("quadcopter")}
    )


##
# Environment configuration
##


@configclass
class QuadcopterEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the cartpole environment."""

    # Scene settings
    scene: QuadcopterSceneCfg = QuadcopterSceneCfg(num_envs=4096,
                                                   env_spacing=1.0)
    seed: int = 0
    
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()

    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 1
        self.episode_length_s = 5.0
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.dt = 0.02
        self.sim.render_interval = self.decimation
