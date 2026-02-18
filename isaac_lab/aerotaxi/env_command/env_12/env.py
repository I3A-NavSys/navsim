from __future__ import annotations

import os
root_navsim_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../..'))

"""Rest everything follows."""

import math
import torch
from scipy.spatial.transform import Rotation

import isaaclab.envs.mdp as mdp
from isaaclab.markers import VisualizationMarkersCfg, VisualizationMarkers
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
from .flight_plan import FlightPlan
# ruido gaussiano para la regularización -- Teresa ---
from isaaclab.utils.noise import GaussianNoiseCfg
# ---------------------------------------------------

# |---------------------------------------------------------|
# |--------------------- ACTIONS ---------------------------|
# |---------------------------------------------------------|

class UAVactionTerm(ActionTerm):
    """Action term for the UAV."""

    _asset: Articulation
    _env: ManagerBasedRLEnv

    def __init__(self, cfg: UAVactionTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._raw_actions = torch.zeros(env.num_envs, 4, device=self.device)
        self._processed_actions = torch.zeros(env.num_envs, 10, 3, device=self.device)
        self.action_scale = 10 # antes 10 
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

        self._asset = env.scene[cfg.asset_name]



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
        # Si algun dron reventó y el NaN se cuela, lo ponemos a 0
        actions = torch.clamp(actions, -2.0, 2.0)
        actions = torch.nan_to_num(actions, nan=0.0)
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
        torch_2 = torch.tensor(2, device=self.device)
        
        # Process raw actions (vectorized)
        self._raw_actions = actions.abs() * self.action_scale


        # print(f"[DEBUG]: raw_actions: {self._raw_actions[0]}")
        
        # Get velocities (assuming these are already tensors)
        lin_vels = torch.nan_to_num(self._asset.data.root_com_lin_vel_b, nan=0.0, posinf=0.0, neginf=0.0)
        ang_vels = torch.nan_to_num(self._asset.data.root_com_ang_vel_b, nan=0.0, posinf=0.0, neginf=0.0)
        # lin_vels = self._env.observation_manager._obs_buffer["policy"][:, :3]  # shape: (num_envs, 3)
        # ang_vels = self._env.observation_manager._obs_buffer["policy"][:, 6:9]  # shape: (num_envs, 3)
        
        # Compute thrust forces (vectorized)
        thrust_coeffs = torch.tensor([kFT_N, kFT_N, kFT_S, kFT_S], device=self.device)
        thrust_z = thrust_coeffs * self._raw_actions**torch_2
        # evito que la componente z sea infinita
        thrust_z = torch.clamp(thrust_z, max=5000.0)
        FT_all = torch.zeros(self._env.num_envs, 4, 3, device=self.device)
        FT_all[:, :, 2] = thrust_z  # Only z-component is non-zero
        
        # Compute drag forces (vectorized)
        FD = -torch.stack([kFDx, kFDy, kFDz]) * lin_vels * lin_vels.abs()
        # limito resistencia al aire
        FD = torch.clamp(FD, -5000.0, 5000.0)


        # Compute drag moments (vectorized)
        MDR_coeffs = torch.tensor([kMDR_N, kMDR_N, kMDR_S, kMDR_S], device=self.device)
        MDR_z = MDR_coeffs * self._raw_actions**torch_2
        MDR = torch.zeros(self._env.num_envs, 3, device=self.device)
        MDR[:, 2] = MDR_z[:, 1] - MDR_z[:, 0] - MDR_z[:, 3] + MDR_z[:, 2]  # NE-NW-SE+SW
        
        # Compute friction moments (vectorized)
        MD = -torch.stack([kMDx, kMDy, kMDz]) * ang_vels * ang_vels.abs()
        # Combine moments (vectorized) and limit it
        torque = torch.clamp(MDR + MD, -5000.0, 5000.0)
        
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

def my_obs_pos(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.root_com_pos_w - env.scene.env_origins # posición relativa

# ----- Teresa -------
def my_obs_dist(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    command_term = env.command_manager.get_term("vel_command")
    
    uav_pos_local = asset.data.root_com_pos_w - env.scene.env_origins
    relative_pos = command_term.target_pos - uav_pos_local[:, :3] # que coja no dónde está con respecto al centro, si no
    # a cuánto está del punto, si no sobreajusta

    # que el dron conozca la orientación a la que está ese punto
    invertir_z = math_utils.quat_inv(asset.data.root_com_quat_w)
    rel_pos_b = math_utils.quat_apply(invertir_z, relative_pos)

    return rel_pos_b # es [dist_x,dist_y,dist_z,z_dron_relativa]
# ------------------------------
def my_obs_height(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    uav_pos_local = asset.data.root_com_pos_w - env.scene.env_origins
    altura_z = uav_pos_local[:, 2:3]
    return altura_z

def my_obs_lin_vel(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.root_com_lin_vel_b

def my_obs_ang_vel(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.root_com_ang_vel_b

def my_obs_roll(env:ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    roll, _, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
    roll = torch.atan2(torch.sin(roll), torch.cos(roll)) # normalize angle to [-pi, pi]
    roll = roll.unsqueeze(1)  # Add a dimension to match the expected shape

    return roll

def my_obs_pitch(env:ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    _, pitch, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
    pitch = torch.atan2(torch.sin(pitch), torch.cos(pitch)) # normalize angle to [-pi, pi]
    pitch = pitch.unsqueeze(1)  # Add a dimension to match the expected shape

    return pitch

def my_obs_gravity_vector(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    quat_w = asset.data.root_com_quat_w

    # eje Z del cuerpo
    z_body = torch.tensor([0.0, 0.0, 1.0], device=quat_w.device)
    z_body = z_body.expand(quat_w.shape[0], 3)

    # lo rotamos al world
    z_world = math_utils.quat_apply(quat_w, z_body)

    return z_world



# Para el crítico
def my_obs_vel_diff(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):    
    asset = env.scene["aerotaxi"]
    command_term = env.command_manager.get_term("vel_command")
    target_vel_w = command_term.target_vel 
    quat_inv = math_utils.quat_inv(asset.data.root_com_quat_w)
    target_vel_b = math_utils.quat_apply(quat_inv, target_vel_w)
    diference = target_vel_b - asset.data.root_com_lin_vel_b 
    return diference

# def my_obs_yaw(env:ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
#     asset: Articulation = env.scene[asset_cfg.name]
#     _, _, yaw = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
#     yaw = torch.atan2(torch.sin(yaw), torch.cos(yaw)) # normalize angle to [-pi, pi]
#     yaw = yaw.unsqueeze(1)  # Add a dimension to match the expected shape

#     return yaw


@configclass
class ObervervationCfg:
    """Observation specifications for the environment."""

    # Actor: lo que el dron verá en train y test
    @configclass
    class PolicyCfg(ObsGroup):
        """Observation group for the policy."""
        # dist = ObsTerm(func=my_obs_dist, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.01))
        # lin_vel = ObsTerm(func=my_obs_lin_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.05))
        # ang_vel = ObsTerm(func=my_obs_ang_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.05))
        # roll = ObsTerm(func=my_obs_roll, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.01))
        # pitch = ObsTerm(func=my_obs_pitch, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.01))
        # yaw = ObsTerm(func=my_obs_yaw, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.01))
        # height = ObsTerm(func=my_obs_height, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.01))
        dist = ObsTerm(func=my_obs_dist, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        lin_vel = ObsTerm(func=my_obs_lin_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        vel_diff = ObsTerm(func=my_obs_vel_diff, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        ang_vel = ObsTerm(func=my_obs_ang_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # roll = ObsTerm(func=my_obs_roll, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # pitch = ObsTerm(func=my_obs_pitch, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # yaw = ObsTerm(func=my_obs_yaw, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        height = ObsTerm(func=my_obs_height, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        z_vect = ObsTerm(func=my_obs_gravity_vector, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})

        def __post_init__(self):
            self.enable_corruption = True  # Regularización con ruido en las observaciones
            self.concatenate_terms = True

    # Crítico: la corrección que se hará sobre lo que ve el dron en train. En test no hay crítico
    # Por eso aquí vamos a incluir la velocidad del punto guía, para que pueda ajustarse a ella en train, pero en test no
    # la vea


    @configclass
    class CriticCfg(ObsGroup):
        """Lo que el entrenador sabe (la verdad absoluta, sin ruido)"""
        # 1. Posición y velocidad de la Policy (pero sin ruido)
        # pos = ObsTerm(func=my_obs_pos, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        dist = ObsTerm(func=my_obs_dist, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        vel_diff = ObsTerm(func=my_obs_vel_diff, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        lin_vel = ObsTerm(func=my_obs_lin_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        ang_vel = ObsTerm(func=my_obs_ang_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # roll = ObsTerm(func=my_obs_roll, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # pitch = ObsTerm(func=my_obs_pitch, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # yaw = ObsTerm(func=my_obs_yaw, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        height = ObsTerm(func=my_obs_height, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        z_vect = ObsTerm(func=my_obs_gravity_vector, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # La nueva función del target también necesita saber respecto a qué dron rotar
        # target_vel = ObsTerm(
        #     func=my_obs_target_vel, 
        #     params={"asset_cfg": SceneEntityCfg(name="aerotaxi")}
        # )
        # target_pos = ObsTerm(func=my_obs_target_pos, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # target_ang_vel = ObsTerm(func=my_obs_target_ang_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # prev_action = ObsTerm(func=my_obs_prev_action, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})

        def __post_init__(self):
            self.enable_corruption = False # El crítico no necesita ruido
            self.concatenate_terms = True
            # self.history_length = 1

    policy: PolicyCfg = PolicyCfg()
    critic: CriticCfg = CriticCfg()



# |---------------------------------------------------------|
# |--------------------- COMMANDS --------------------------|
# |---------------------------------------------------------|

class UAVcommandTerm(CommandTerm):
    """Command term for the UAV that generates meaningful velocity and yaw rate commands."""
    
    _asset: Articulation
    
    def __init__(self, cfg: UAVcommandTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._marker_visualizer = VisualizationMarkers(VISUAL_TARGET_CFG)
        self._asset = env.scene[cfg.asset_name]
        
        # ponemos la velocidad y giro al que deberá ir nuestro punto guía
        # velocidad en x,y,z (tamaño 3)
        self.target_vel = torch.zeros(self.num_envs, 3, device=self.device)
        self.target_yaw = torch.zeros(self.num_envs,device=self.device)

        # estado del punto guía en t (vel_x,vel_y,vel_z, yaw_rate)
        self._command = torch.zeros(self.num_envs,4,device=self.device)

        # posición del punto guía (x,y,z, tamaño 3)
        self.target_pos = torch.zeros(self.num_envs, 3, device=self.device)
        # t actual
        self.dt = env.step_dt 

    @property
    def command(self) -> torch.Tensor:
        return self._command
    
    def _update_metrics(self):
        pass


    def _resample_command(self, env_ids: torch.Tensor):
        # primer frame de tiempo
        # if getattr(self._env.cfg, "is_test_mode", False):
        #     pass
        # else:
        #     pass
        
        # El objetivo empieza en la posición actual del dron para un despegue suave
        self._asset.update(self.dt)
        uav_pos_w = self._asset.data.root_com_pos_w[env_ids] - self._env.scene.env_origins[env_ids]
        self.target_pos[env_ids] = uav_pos_w[:, :3]
        self.target_vel[env_ids] = 0.0
        self.target_yaw[env_ids] = 0.0


    def _update_command(self):
        # # 5. Comando para la red (Posición relativa en Body Frame)
        uav_pos_local = self._asset.data.root_com_pos_w[:, :3] - self._env.scene.env_origins[:, :3]
        rel_pos_w = self.target_pos - uav_pos_local
        quat_inv = math_utils.quat_inv(self._asset.data.root_com_quat_w)
        self._command[:, :3] = math_utils.quat_apply(quat_inv, rel_pos_w)
        # # ---------------------------
        # # Visualización dinámica
        target_pos_w = self.target_pos + self._env.scene.env_origins
        self._marker_visualizer.visualize(translations=target_pos_w)
@configclass
class UAVcommandTermCfg(CommandTermCfg):
    """Command term configuration for the UAV."""

    class_type: type = UAVcommandTerm
    """Class type of the command term."""
    asset_name: str = "aerotaxi"

@configclass
class CommandCfg:
    """Command specifications for the environment."""
    
    vel_command = UAVcommandTermCfg(asset_name="aerotaxi",resampling_time_range=(15, 15)) # cada 15 segundos cambiamos


# |---------------------------------------------------------|
# |--------------------- EVENTS ----------------------------|
# |---------------------------------------------------------|

@configclass
class EventCfg:
    """Event specifications for the environment."""

    # DOMAIN RANDOMIZATION: posición, velocidad, roll, pitch, yaw, masa y velocidad del viento
    reset_pos = EventTerm(
        func=mdp.reset_root_state_uniform, 
        mode="reset",
        params={
            "pose_range": {
                "x": (-5.0, 5.0), 
                "y": (-5.0, 5.0), 
                "z": (8.0, 12.0),
                "roll": (-0.5, 0.5),
                "pitch": (-0.5, 0.5),
                "yaw": (-3.14, 3.14)
            },
            "velocity_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (-0.5, 0.5)
            },
            "asset_cfg": SceneEntityCfg(name="aerotaxi")
        }
    )

    # # Masa: 0.9 su masa y 1.1 su masa (con pasajeros, o por si alguno es especialmente menos pesado)
    # randomize_mass = EventTerm(
    #     func=mdp.randomize_rigid_body_mass,
    #     mode="reset",
    #     params={
    #         "asset_cfg": SceneEntityCfg(name="aerotaxi"),
    #         "mass_distribution_params": (0.9, 1.1), 
    #         "operation": "scale"
    #     }
    # )
    # # Viento: el dron recibirá una corriente de aire en diversas direcciones de forma aleatoria.
    # randomize_wind = EventTerm(
    #     func=mdp.push_by_setting_velocity,
    #     mode="reset",
    #     params={
    #         "asset_cfg": SceneEntityCfg(name="aerotaxi"),
    #         "velocity_range": {
    #             "x": (-1.0, 1.0), 
    #             "y": (-1.0, 1.0),
    #             "z": (-0.5, 0.5)
    #         }
    #     }
    # )


# |---------------------------------------------------------|
# |--------------------- REWARDS ---------------------------|
# |---------------------------------------------------------|

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""
    # alive = RewTerm(func=mdp.is_alive, weight=15.0)

    action_rate = RewTerm(func=my_rewards.rew_action_rate, weight=-0.01)

    terminating = RewTerm(func=mdp.is_terminated, weight=-300.0)

    pos_reward = RewTerm(
        func=my_rewards.rew_pos_diff,
        weight=-15.0,
    )
    rew_lin_vel_diff = RewTerm(
        func=my_rewards.rew_lin_vel_diff,
        weight=8.0,
    )
    tilt_penalty = RewTerm(
        func=my_rewards.rew_tilt_penalty,
        weight=-5.0,
    )

    # rew_pos_diff_fine_grained = RewTerm(
    #     func=my_rewards.rew_pos_diff_fine_grained,
    #     weight=10.0,
    #     params={"std": 5.0},
    # )

    # rew_lin_vel_diff_fine_grained = RewTerm(
    #     func=my_rewards.rew_lin_vel_diff_fine_grained,
    #     weight=4.0,
    #     params={"std": 1.0},
    # )

    # rew_ang_vel_z_diff_fine_grained = RewTerm(
    #     func=my_rewards.rew_ang_vel_z_diff_fine_grained,
    #     weight=2.0,
    #     params={"std": 0.25},
    # )

    # rew_roll_diff_fine_grained = RewTerm(
    #     func=my_rewards.rew_roll_diff_fine_grained,
    #     weight=4.0,
    #     params={"std": 0.2, "target": 0.0}, # 0.5 para que pueda girarse un poco el ángulo y siga obteniendo reward
    # )
    # rew_pitch_diff_fine_grained = RewTerm(
    #     func=my_rewards.rew_pitch_diff_fine_grained,
    #     weight=4.0,
    #     params={"std": 0.2, "target": 0.0}, # 0.5 para que pueda girarse un poco el ángulo y siga obteniendo reward
    # )
    # rew_ang_vel_xy_penalty = RewTerm(
    #     func=my_rewards.rew_ang_vel_xy_penalty, 
    #     weight=-0.5 
    # )
    # rew_vel_rescue = RewTerm(
    #     func=my_rewards.rew_velocity_rescue_bidirectional,
    #     weight=2.0, 
    # )

# |---------------------------------------------------------|
# |--------------------- TERMINATIONS ----------------------|
# |---------------------------------------------------------|

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    time_out = DoneTerm(func=mdp.time_out, time_out=True)

    below_min_altitude = DoneTerm(
        func=my_terminations.below_min_altitude,
        params={"min_altitude": 0.2,} # el centro de masas del dron está a 5.06m del suelo.
    )
    bad_attitude = DoneTerm(func=my_terminations.roll_pitch_termination)
    safety_shutdown = DoneTerm(func=my_terminations.are_nan_or_exploded)


# |---------------------------------------------------------|
# |--------------------- SCENE -----------------------------|
# |---------------------------------------------------------|

VISUAL_TARGET_CFG = VisualizationMarkersCfg(
    prim_path="/Visuals/target_commands",
    markers={
        "target": sim_utils.SphereCfg(
            radius=1.0,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)), # Rojo
        ),
    },
)

@configclass
class MySceneCfg(InteractiveSceneCfg):
    """Configuration for a UAV scene"""

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(6000, 6000))
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
            pos=(0, 0, 1.75),
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
    scene: MySceneCfg = MySceneCfg(env_spacing=200.0, replicate_physics=False)
    seed: int = 0
    
    # Basic settings
    actions: ActionsCfg = ActionsCfg()
    observations: ObervervationCfg = ObervervationCfg()
    commands: CommandCfg = CommandCfg()
    events: EventCfg = EventCfg()

    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    is_test_mode: bool = False

    def __post_init__(self):
        """Post initialization"""
        # ----- Teresa -------
        # pasos que se usan en la red de PPO
        self.num_steps_per_env = 100
        # ----- Teresa -------

        # viewer settings
        self.viewer.eye = [4.5, 0.0, 6.0]
        self.viewer.lookat = [0.0, 0.0, 2.0]
        # step settings
        self.decimation = 4  # 50 Hz de actualización para la IA
        self.episode_length_s = 25.0 # al ser punto cambiante no debe ser tan largo
        # simulation settings
        self.sim.dt = 0.005  # 100 Hz para las físicas
        self.sim.render_interval = self.decimation
        