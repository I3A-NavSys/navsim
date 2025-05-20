from __future__ import annotations

import os
root_navsim_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))

import math
import torch

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, Articulation, ArticulationCfg
from isaaclab.envs import ManagerBasedRLEnv, ManagerBasedRLEnvCfg
from isaaclab.managers import ActionTermCfg, ActionTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import CommandTermCfg, CommandTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.actuators import DCMotorCfg
import isaaclab.utils.math as math_utils
from . import rewards as my_rewards
from . import terminations as my_terminations


# |---------------------------------------------------------|
# |--------------------- ACTIONS ---------------------------|
# |---------------------------------------------------------|

class UAVactionTerm(ActionTerm):
    """Action term for the UAV."""

    _asset: Articulation

    def __init__(self, cfg: UAVactionTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._raw_actions = torch.zeros(env.num_envs, 4, device=self.device)
        self._processed_actions = torch.zeros(env.num_envs, 10, 3, device=self.device)
        self.action_scale = 10
        self.max_prim_links = 5 # 4 rotors + 1 body

        # Create all positions at once in a single tensor operation
        position_data = torch.tensor([
            [0, 0, 0],
            [0.5, 1.95, 0.5],
            [0.5, -1.95, 0.5],
            [-2.5, 1.55, 0.5],
            [-2.5, -1.55, 0.5]
        ], device=self.device)

        # Expand to all environments (no copy, just view)
        self.positions = position_data.unsqueeze(0).expand(env.num_envs, -1, -1).clone()

        # Create indexes efficiently
        self.indexes = torch.arange(env.num_envs, device=self.device)


    @property
    def action_dim(self) -> int:
        return self._raw_actions.shape[1]

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions: torch.Tensor):
        # Define constants as tensors
        kFT_N = torch.tensor(4.6544, device=self.device)
        kFT_S = torch.tensor(0.9309, device=self.device)
        kFDx = torch.tensor(3.0625, device=self.device)
        kFDy = torch.tensor(4.0000, device=self.device)
        kFDz = torch.tensor(7.8400, device=self.device)
        kMDR_N = torch.tensor(5.9683, device=self.device)
        kMDR_S = torch.tensor(1.4921, device=self.device)
        kMDx = torch.tensor(37.4010, device=self.device)
        kMDy = torch.tensor(25.8580, device=self.device)
        kMDz = torch.tensor(20.2514, device=self.device)
        
        # Process raw actions (vectorized)
        self._raw_actions = actions.abs() * self.action_scale

        # print(f"[DEBUG]: raw_actions: {self._raw_actions[0]}")
        
        # Get velocities (assuming these are already tensors)
        lin_vels = self._asset.data.root_com_lin_vel_b  # shape: (num_envs, 3)
        ang_vels = self._asset.data.root_com_ang_vel_b  # shape: (num_envs, 3)
        
        # Compute thrust forces (vectorized)
        thrust_coeffs = torch.tensor([kFT_N, kFT_N, kFT_S, kFT_S], device=self.device)
        thrust_z = thrust_coeffs * self._raw_actions**2
        FT_all = torch.zeros(self._env.num_envs, 4, 3, device=self.device)
        FT_all[:, :, 2] = thrust_z  # Only z-component is non-zero
        
        # Compute drag forces (vectorized)
        FD = -torch.stack([kFDx, kFDy, kFDz]) * lin_vels * lin_vels.abs()
        
        # Compute drag moments (vectorized)
        MDR_coeffs = torch.tensor([kMDR_N, kMDR_N, kMDR_S, kMDR_S], device=self.device)
        MDR_z = MDR_coeffs * self._raw_actions**2
        MDR = torch.zeros(self._env.num_envs, 3, device=self.device)
        MDR[:, 2] = MDR_z[:, 1] - MDR_z[:, 0] - MDR_z[:, 3] + MDR_z[:, 2]  # NE-NW-SE+SW
        
        # Compute friction moments (vectorized)
        MD = -torch.stack([kMDx, kMDy, kMDz]) * ang_vels * ang_vels.abs()
        
        # Combine moments (vectorized)
        torque = MDR + MD
        zero_torque = torch.zeros_like(torque)
        
        # Build processed actions tensor (vectorized)
        self._processed_actions[:, 0] = FD  # Drag force
        self._processed_actions[:, 1] = FT_all[:, 0]  # FT_NW
        self._processed_actions[:, 2] = FT_all[:, 1]  # FT_NE
        self._processed_actions[:, 3] = FT_all[:, 2]  # FT_SW
        self._processed_actions[:, 4] = FT_all[:, 3]  # FT_SE
        self._processed_actions[:, 5] = torque  # Combined torque
        self._processed_actions[:, 6:] = zero_torque.unsqueeze(1).expand(-1, 4, -1)  # Zero torques

    def apply_actions(self):
        self._asset.root_physx_view.apply_forces_and_torques_at_position(
            force_data=self._processed_actions[:, :self.max_prim_links, :], 
            torque_data=self._processed_actions[:, self.max_prim_links:, :],
            position_data=self.positions,
            indices=self.indexes,
            is_global=False
        )

        # This can be used to simulate wind forces it seems
        # mdp.apply_external_force_torque()

@configclass
class UAVactionTermCfg(ActionTermCfg):
    """Action term configuration for the UAV."""

    class_type: type = UAVactionTerm
    """Class type of the action term."""

@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    rotors_vel = UAVactionTermCfg(asset_name="aerotaxi")


# |---------------------------------------------------------|
# |--------------------- OBSERVATIONS ----------------------|
# |---------------------------------------------------------|

def my_obs_lin_vel(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    lin_vel = asset.data.root_com_lin_vel_b
    return lin_vel

def my_obs_ang_vel(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    ang_vel = asset.data.root_com_ang_vel_b
    return ang_vel

def my_obs_roll(env:ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    roll, _, _ = math_utils.euler_xyz_from_quat(asset.data.root_quat_w)
    roll = torch.atan2(torch.sin(roll), torch.cos(roll)) # normalize angle to [-pi, pi]
    roll = roll.unsqueeze(1)  # Add a dimension to match the expected shape

    return roll

def my_obs_pitch(env:ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    _, pitch, _ = math_utils.euler_xyz_from_quat(asset.data.root_quat_w)
    pitch = torch.atan2(torch.sin(pitch), torch.cos(pitch)) # normalize angle to [-pi, pi]
    pitch = pitch.unsqueeze(1)  # Add a dimension to match the expected shape

    return pitch

def my_obs_command(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Get current velocity commands."""
    return env.command_manager.get_command("vel_command")

def my_obs_command_error(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Calculate error between current state and commanded values."""
    asset: Articulation = env.scene[asset_cfg.name]
    
    # Get current state
    lin_vel = asset.data.root_com_lin_vel_b
    ang_vel = asset.data.root_com_ang_vel_b
    command = env.command_manager.get_command("vel_command")
    
    # Calculate errors
    lin_vel_error = lin_vel - command[:, :3]
    ang_vel_error = ang_vel[:, 2] - command[:, 3]  # Only yaw for UAV
    
    return torch.cat([lin_vel_error, ang_vel_error.unsqueeze(1)], dim=1)

@configclass
class ObervervationCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observation group for the policy."""
        lin_vel = ObsTerm(func=my_obs_lin_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        ang_vel = ObsTerm(func=my_obs_ang_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        roll = ObsTerm(func=my_obs_roll, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        pitch = ObsTerm(func=my_obs_pitch, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        current_command = ObsTerm(func=my_obs_command)
        command_error = ObsTerm(func=my_obs_command_error, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        
        def __post_init__(self):
            self.enable_corruption = False  # Commands should never be corrupted
            self.concatenate_terms = True

    policy: PolicyCfg = PolicyCfg()


# |---------------------------------------------------------|
# |--------------------- COMMANDS --------------------------|
# |---------------------------------------------------------|

class UAVcommandTerm(CommandTerm):
    """Command term for the UAV."""

    _asset: Articulation

    def __init__(self, cfg: UAVcommandTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._command = torch.zeros(env.num_envs, 4, device=self.device)

    @property
    def command(self) -> torch.Tensor:
        """The command tensor. Shape is (num_envs, command_dim)."""
        return self._command
    
    def _update_metrics(self):
        pass

    def _resample_command(self, env_ids):
        """Resample the command for the given environment IDs."""
        self._command[env_ids, :] = torch.zeros(4, device=self.device)

    def _update_command(self):
        # self._command[:] = torch.zeros(4, device=self.device)
        pass

@configclass
class UAVcommandTermCfg(CommandTermCfg):
    """Command term configuration for the UAV."""

    class_type: type = UAVcommandTerm
    """Class type of the command term."""

@configclass
class CommandCfg:
    """Command specifications for the environment."""

    vel_command = UAVcommandTermCfg(resampling_time_range=(10, 10))


# |---------------------------------------------------------|
# |--------------------- EVENTS ----------------------------|
# |---------------------------------------------------------|

@configclass
class EventCfg:
    """Event specifications for the environment."""

    reset_pos = EventTerm(
        func=mdp.reset_root_state_uniform, 
        mode="reset",
        params={
            "pose_range": {
                "x": (0, 0), 
                "y": (0, 0), 
                # "roll": (0, 0),
                # "pitch": (0, 0),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-3.14, 3.14)
            },
            "velocity_range": {
                "x": (-2, 2),
                "y": (-2, 2),
                "z": (-2, 2)
            },
            "asset_cfg": SceneEntityCfg(name="aerotaxi")
        }
    )


# |---------------------------------------------------------|
# |--------------------- REWARDS ---------------------------|
# |---------------------------------------------------------|

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""
    alive = RewTerm(func=mdp.is_alive, weight=2.0)

    terminating = RewTerm(func=mdp.is_terminated, weight=-1000.0)

    rew_x_lin_vel_diff = RewTerm(
        func=my_rewards.rew_x_lin_vel_diff,
        weight=1.0,
    )

    rew_y_lin_vel_diff = RewTerm(
        func=my_rewards.rew_y_lin_vel_diff,
        weight=1.0,
    )

    rew_z_lin_vel_diff = RewTerm(
        func=my_rewards.rew_z_lin_vel_diff,
        weight=1.0,
    )

    rew_z_ang_vel_diff = RewTerm(
        func=my_rewards.rew_z_ang_vel_diff,
        weight=1.0,
    )

    pen_jerky_mov = RewTerm(
        func=my_rewards.pen_jerky_mov,
        weight=-0.1,
    )

    roll = RewTerm(
        func=my_rewards.pen_roll_diff,
        weight=-10.0,
        params={"target": 0.0},
    )

    pitch = RewTerm(
        func=my_rewards.pen_pitch_diff,
        weight=-10.0,
        params={"target": 0.0},
    )


# |---------------------------------------------------------|
# |--------------------- TERMINATIONS ----------------------|
# |---------------------------------------------------------|

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # (3) Z position out of bounds
    below_min_altitude = DoneTerm(
        func=my_terminations.below_min_altitude,
        params={"asset_cfg": SceneEntityCfg("aerotaxi", joint_names=["NW_joint", "NE_joint", "SW_joint", "SE_joint"]), 
                "min_altitude": 10.0,
        }
    )


# |---------------------------------------------------------|
# |--------------------- SCENE -----------------------------|
# |---------------------------------------------------------|

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
                max_linear_velocity=20.0,
                max_angular_velocity=572.95779578552,
                max_depenetration_velocity=10.0,
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


# |---------------------------------------------------------|
# |--------------------- ENVIRONMENT -----------------------|
# |---------------------------------------------------------|

@configclass
class UAVEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the UAV environment."""

    # Scene settings
    scene: MySceneCfg = MySceneCfg(num_envs=32, env_spacing=10, replicate_physics=False)
    seed: int = 0
    
    # Basic settings
    actions: ActionsCfg = ActionsCfg()
    observations: ObervervationCfg = ObervervationCfg()
    commands: CommandCfg = CommandCfg()
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
        self.decimation = 1  # env step every 4 sim steps: 200Hz / 4 = 50Hz
        self.episode_length_s = 10.0  # 10s
        # simulation settings
        self.sim.dt = 0.02  # sim step every 5ms: 200Hz
        self.sim.render_interval = self.decimation