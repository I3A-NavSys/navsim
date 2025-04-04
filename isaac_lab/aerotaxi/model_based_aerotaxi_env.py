from __future__ import annotations

import os
root_navsim_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../..'))

"""Rest everything follows."""

import math
import torch

# import isaaclab.envs.mdp as mdp
from . import mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, Articulation, ArticulationCfg
from isaaclab.envs import ManagerBasedRLEnv, ManagerBasedRLEnvCfg
from isaaclab.managers import ActionTermCfg, ActionTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.actuators import DCMotorCfg
import isaaclab.utils.math as math_utils
from .aerotaxi_env import UAVEnvCfg

LOW_LEVEL_ENV_CFG = UAVEnvCfg()

@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    pre_trained_policy: mdp.PreTrainedPolicyActionCfg = mdp.PreTrainedPolicyActionCfg(
        asset_name="aerotaxi",
        policy_path=os.path.join(root_navsim_path, "isaac_lab", "exported", "policy.pt"),
        low_level_decimation=4,
        low_level_actions=LOW_LEVEL_ENV_CFG.actions.uav_pos,
        low_level_observations=LOW_LEVEL_ENV_CFG.observations.policy,
    )


def my_obs_pos(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    pos = asset.data.root_pos_w
    return pos

def my_obs_lin_vel(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    lin_vel = asset.data.root_com_lin_vel_b
    return lin_vel

def my_obs_ang_vel(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    ang_vel = asset.data.root_com_ang_vel_b
    return ang_vel

def my_obs_ori(env:ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    ori = torch.zeros(env.num_envs, 3, device=env.device)

    roll, pitch, yaw = math_utils.euler_xyz_from_quat(asset.data.root_quat_w)
    
    # # normalize angle to [-pi, pi]
    roll = torch.atan2(torch.sin(roll), torch.cos(roll))
    pitch = torch.atan2(torch.sin(pitch), torch.cos(pitch))
    yaw = torch.atan2(torch.sin(yaw), torch.cos(yaw))

    ori[:, 0] = roll
    ori[:, 1] = pitch
    ori[:, 2] = yaw

    return ori

@configclass
class ObervervationCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observation group for the policy."""
        pos = ObsTerm(func=my_obs_pos, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        lin_vel = ObsTerm(func=my_obs_lin_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        ang_vel = ObsTerm(func=my_obs_ang_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        ori = ObsTerm(func=my_obs_ori, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})

        def __pos_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()

@configclass
class EventCfg:
    """Event specifications for the environment."""

    reset_pos = EventTerm(
        func=mdp.reset_root_state_uniform, 
        mode="reset",
        params={
            "pose_range": {
                "x": (-1, 1), 
                "y": (-1, 1), 
                # "roll": (0, 0),
                # "pitch": (0, 0),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-3.14, 3.14)
            },
            "velocity_range": {
                "x": (0, 0),
                "y": (0, 0),
                "z": (0, 0)
            },
            "asset_cfg": SceneEntityCfg(name="aerotaxi")
        }
    )

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # (1) Constant running reward
    alive = RewTerm(func=mdp.is_alive, weight=2.0)
    # (2) Failure penalty
    terminating = RewTerm(func=mdp.is_terminated, weight=-400.0)
    # (3) Primary task: modern control
    modern_control = RewTerm(
        func=mdp.modern_control_diff,
        weight=2.0,
        params={
            "asset_cfg": SceneEntityCfg("aerotaxi", joint_names=["NW_joint", "NE_joint", "SW_joint", "SE_joint"]),
            "target": [0.0, 0.0, 0.0, 0.0]
        },
    )

    hover = RewTerm(
        func=mdp.lin_vel_diff,
        weight=1.0,
        params={
            "asset_cfg": SceneEntityCfg("aerotaxi", joint_names=["NW_joint", "NE_joint", "SW_joint", "SE_joint"]), 
            "target": [0.0, 0.0, 0.0]
        },
    )

    falling = RewTerm(
        func=mdp.lin_vel_z_diff,
        weight=-5.0,
        params={
            "asset_cfg": SceneEntityCfg("aerotaxi", joint_names=["NW_joint", "NE_joint", "SW_joint", "SE_joint"]), 
            "target": 0.0
        },
    )

    rotation = RewTerm(
        func=mdp.ang_vel_diff,
        weight=-10.0,
        params={
            "asset_cfg": SceneEntityCfg("aerotaxi", joint_names=["NW_joint", "NE_joint", "SW_joint", "SE_joint"]), 
            "target": [0.0, 0.0, 0.0]
        },
    )

    roll = RewTerm(
        func=mdp.roll_diff,
        weight=-10.0,
        params={
            "asset_cfg": SceneEntityCfg("aerotaxi", joint_names=["NW_joint", "NE_joint", "SW_joint", "SE_joint"]), 
            "target": 0.0
        },
    )

    pitch = RewTerm(
        func=mdp.pitch_diff,
        weight=-10.0,
        params={
            "asset_cfg": SceneEntityCfg("aerotaxi", joint_names=["NW_joint", "NE_joint", "SW_joint", "SE_joint"]), 
            "target": 0.0
        },
    )

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # (2) Linear velocity in z direction exceeds a negative threshold
    len_vel_z_out_bounds = DoneTerm(
        func=mdp.lin_vel_z_termination,
        params={"asset_cfg": SceneEntityCfg("aerotaxi", joint_names=["NW_joint", "NE_joint", "SW_joint", "SE_joint"]), 
        }
    )
    # (3) Z position out of bounds
    below_min_altitude = DoneTerm(
        func=mdp.below_min_altitude,
        params={"asset_cfg": SceneEntityCfg("aerotaxi", joint_names=["NW_joint", "NE_joint", "SW_joint", "SE_joint"]), 
                "min_altitude": 10.0,
        }
    )

@configclass
class MySceneCfg(InteractiveSceneCfg):
    """Configuration for a UAV scene"""

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100, 100))
    )

    aerotaxi: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/aerotaxi",
        spawn=sim_utils.UsdFileCfg(
            usd_path=os.path.abspath(os.path.join(root_navsim_path, "isaac_lab", "aerotaxi", "UAM_aerotaxi_lab.usd")),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                rigid_body_enabled=True,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=100.0,
                enable_gyroscopic_forces=True,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
                sleep_threshold=0.005,
                stabilization_threshold=0.001,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0, 0, 50),
            joint_pos={
                "NW_joint": 0.0,
                "NE_joint": 0.0,
                "SW_joint": 0.0,
                "SE_joint": 0.0,
            },
        ),
        actuators={
            "NW_rotor": DCMotorCfg(
                joint_names_expr=["NW_joint"],
                effort_limit=100000.0,
                velocity_limit=100000.0,
                stiffness=0.0,
                damping=0.0,
                saturation_effort=100000.0,
            ),
            "NE_rotor": DCMotorCfg(
                joint_names_expr=["NE_joint"],
                effort_limit=100000.0,
                velocity_limit=100000.0,
                stiffness=0.0,
                damping=0.0,
                saturation_effort=100000.0,
            ),
            "SW_rotor": DCMotorCfg(
                joint_names_expr=["SW_joint"],
                effort_limit=100000.0,
                velocity_limit=100000.0,
                stiffness=0.0,
                damping=0.0,
                saturation_effort=100000.0,
            ),
            "SE_rotor": DCMotorCfg(
                joint_names_expr=["SE_joint"],
                effort_limit=100000.0,
                velocity_limit=100000.0,
                stiffness=0.0,
                damping=0.0,
                saturation_effort=100000.0,
            ),
        }
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(
            color=(0.75, 0.75, 0.75),
            intensity=30000
        )
    )

@configclass
class UAVEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the UAV environment."""

    # Scene settings
    scene: MySceneCfg = MySceneCfg(num_envs=32, env_spacing=10, replicate_physics=False)
    
    # Basic settings
    observations: ObervervationCfg = ObervervationCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        """Post initialization"""
        # viewer settings
        self.viewer.eye = [4.5, 0.0, 6.0]
        self.viewer.lookat = [0.0, 0.0, 2.0]
        # step settings
        self.decimation = 4  # env step every 4 sim steps: 200Hz / 4 = 50Hz
        self.episode_length_s = 10.0  # 10s
        # simulation settings
        self.sim.dt = 0.005  # sim step every 5ms: 200Hz
        self.sim.render_interval = self.decimation