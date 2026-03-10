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
# para la flecha
# -------------------------------------------------
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
# -------------------------------------------------
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
# para las flechas
from isaaclab.utils.math import quat_from_matrix
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
        self.action_scale = 30 # antes 10 
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
        # -------------- Teresa  ----------------------------------
        actions = torch.clamp(actions, -1.0, 1.0) # antes -2,2
        actions = torch.nan_to_num(actions, nan=0.0)
        # Define constants as tensors

        # Vamos a pasarle equilibrio físico directamente para que:
        # 0 - hover; >0 - se eleve; <0 - baje
        # ----------------------- intento 1-----------------------------
        # mass = self._asset.data.default_mass.sum(dim=1, keepdim=True)
        # weight = mass*9.81 # gravedad
        # thrust_hover_total = weight
        # thrust_hover_per_rotor = torch.tensor(thrust_hover_total / 4.0, device=self.device)
        # kFT = torch.tensor([4.6544, 4.6544, 0.9309, 0.9309], device=self.device)
        # raw_hover = torch.sqrt(thrust_hover_per_rotor / kFT)

        # thrust_scale = 0.3
        # self._raw_actions = raw_hover * (1.0 + thrust_scale * actions)
        # self._raw_actions = torch.clamp(self._raw_actions, min=0.0)
        # ------------------------intento 2-----------------------

        mass = self._asset.data.default_mass.sum(dim=1, keepdim=True)
        mg = mass * 9.81

        # posiciones rotores (sin el cuerpo)
        rotor_pos = self.positions[:, 1:5, :]  # (envs,4,3)
        x = rotor_pos[:, :, 0]
        y = rotor_pos[:, :, 1]

        # coeficientes
        kFT = torch.tensor([4.6544, 4.6544, 0.9309, 0.9309], device=self.device)
        kMDR = torch.tensor([5.9683, -5.9683, 1.4921, -1.4921], device=self.device)
        # signos alternados como en tu fórmula yaw

        # Construir matriz A para cada env
        A = torch.zeros(self._env.num_envs, 4, 4, device=self.device)

        # Fuerza total
        A[:, 0, :] = kFT

        # Momento X (roll)
        A[:, 1, :] = y * kFT

        # Momento Y (pitch)
        A[:, 2, :] = -x * kFT

        # Momento Z (yaw)
        A[:, 3, :] = kMDR

        # Vector objetivo
        b = torch.zeros(self._env.num_envs, 4, device=self.device)
        b[:, 0] = mg.squeeze()

        # Resolver sistema
        u_hover = torch.linalg.solve(A, b)

        # Asegurar positivo
        u_hover = torch.clamp(u_hover, min=0.0)

        omega_hover = torch.sqrt(u_hover)
        thrust_scale = 0.5
        self._raw_actions = omega_hover * (1.0 + thrust_scale * actions)
        self._raw_actions = torch.clamp(self._raw_actions, min=0.0)
        # ------------------------------------------------------

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
        
        # # Process raw actions (vectorized)
        # -----------------------------------------------
        # self._raw_actions = actions.abs() * self.action_scale
        # -----------------------------------------------

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
        thrust_z = torch.clamp(thrust_z, max=15000.0) # antes 5000 N, pero es insuficiente para 2200 kilos
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

def my_obs_projected_gravity(env:ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    return asset.data.projected_gravity_b

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

def my_obs_dist2(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    command_term = env.command_manager.get_term("vel_command")
    
    uav_pos_local = asset.data.root_com_pos_w - env.scene.env_origins
    rel_pos_b = command_term.target_pos - uav_pos_local[:, :3] # que coja no dónde está con respecto al centro, si no
    # a cuánto está del punto, si no sobreajusta

    # que el dron conozca la orientación a la que está ese punto
    # invertir_z = math_utils.quat_inv(asset.data.root_com_quat_w)
    # rel_pos_b = math_utils.quat_apply(invertir_z, relative_pos)

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

def my_obs_target_vel(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset = env.scene["aerotaxi"]
    command_term = env.command_manager.get_term("vel_command")
    target_vel_w = command_term.target_vel 
    quat_inv = math_utils.quat_inv(asset.data.root_com_quat_w)
    target_vel_b = math_utils.quat_apply(quat_inv, target_vel_w)
    
    return target_vel_b

def my_obs_yaw(env:ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    _, _, yaw = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
    yaw = torch.atan2(torch.sin(yaw), torch.cos(yaw)) # normalize angle to [-pi, pi]
    yaw = yaw.unsqueeze(1)  # Add a dimension to match the expected shape

    return yaw

def my_obs_target_yaw_rate(env:ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    term = env.command_manager.get_term("vel_command")
    yaw_rate = term.target_yaw
    return yaw_rate.unsqueeze(1) / 0.1

def my_obs_yaw_error(env:ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    _, _, yaw = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)

    # velocidad objetivo del punto guía
    term = env.command_manager.get_term("vel_command")
    target_vel = term.target_vel[:, :2]

    # yaw deseado (dirección de movimiento del target)
    target_yaw = torch.atan2(target_vel[:, 1], target_vel[:, 0])

    # error
    yaw_error = target_yaw - yaw

    # normalizar a [-pi, pi]
    yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

    return yaw_error.unsqueeze(1) / math.pi

def my_obs_target_ang_vel(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    """Velocidad angular deseada en el frame del dron (body frame)."""
    asset = env.scene["aerotaxi"]
    command_term = env.command_manager.get_term("vel_command")
    zeros = torch.zeros_like(command_term.target_pos) # [N, 3]
    zeros[:, 2] = command_term.target_yaw # Ponemos el comando en la componente Z
    
    # 2. Rotamos ese vector al body frame
    quat_inv = math_utils.quat_inv(asset.data.root_com_quat_w)
    target_ang_vel_b = math_utils.quat_apply(quat_inv, zeros)
    
    return target_ang_vel_b


def my_obs_target_pos(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    """Vector hacia el objetivo relativo al dron, sin dar la posición absoluta de entrenamiento."""
    asset = env.scene["aerotaxi"]
    command_term = env.command_manager.get_term("vel_command")
    target_pos_w = command_term.target_pos
    current_pos_w = asset.data.root_com_pos_w[:, :3]
    # Relativo al dron, no al mundo
    target_rel = target_pos_w - current_pos_w
    # Opcional: proyectar al frame del dron si quieres coherencia con velocities
    quat_inv = math_utils.quat_inv(asset.data.root_com_quat_w)
    target_rel_b = math_utils.quat_apply(quat_inv, target_rel)
    return target_rel_b


def my_obs_prev_action(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    """Retorna la acción previa aplicada, útil para rew_action_rate."""
    return env.action_manager.prev_action


def my_obs_command(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Get current velocity commands."""
    return env.command_manager.get_command("vel_command")

@configclass
class ObervervationCfg:
    """Observation specifications for the environment."""

    # Actor: lo que el dron verá en train y test
    @configclass
    class PolicyCfg(ObsGroup):
        """Observation group for the policy."""
        dist = ObsTerm(func=my_obs_dist2, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.01))
        lin_vel = ObsTerm(func=my_obs_lin_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.05))
        ang_vel = ObsTerm(func=my_obs_ang_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.05))
        projected_gravity = ObsTerm(func=my_obs_projected_gravity, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.01))
        target_vel = ObsTerm(func=my_obs_target_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.01)) 
        # pos = ObsTerm(func=my_obs_pos, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.01))
        # yaw_error = ObsTerm(func=my_obs_yaw_error, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.01))
        # target_yaw_rate = ObsTerm(func=my_obs_target_yaw_rate, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.01))
        # prev_action = ObsTerm(func=my_obs_prev_action, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # roll = ObsTerm(func=my_obs_roll, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.01))
        # pitch = ObsTerm(func=my_obs_pitch, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.01))
        # yaw = ObsTerm(func=my_obs_yaw, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.01))

        # current_command = ObsTerm(func=my_obs_command) # habría fuga de datos si no
        # GaussianNoiseCFG: simula el ruido de los sensores, así es como si fuera Regularización
        # target_pos = ObsTerm(func=my_obs_target_pos, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # target_ang_vel = ObsTerm(func=my_obs_target_ang_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # height = ObsTerm(func=my_obs_height, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")},noise=GaussianNoiseCfg(std=0.01))

        def __post_init__(self):
            self.enable_corruption = True  # Regularización con ruido en las observaciones
            self.concatenate_terms = True
            self.history_length = 3 
            self.flatten_history_dim = True

    # Crítico: la corrección que se hará sobre lo que ve el dron en train. En test no hay crítico
    # Por eso aquí vamos a incluir la velocidad del punto guía, para que pueda ajustarse a ella en train, pero en test no
    # la vea


    @configclass
    class CriticCfg(ObsGroup):
        """Lo que el entrenador sabe (la verdad absoluta, sin ruido)"""
        # 1. Posición y velocidad de la Policy (pero sin ruido)
        # pos = ObsTerm(func=my_obs_pos, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        dist = ObsTerm(func=my_obs_dist2, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        lin_vel = ObsTerm(func=my_obs_lin_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        ang_vel = ObsTerm(func=my_obs_ang_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        projected_gravity = ObsTerm(func=my_obs_projected_gravity, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        target_vel = ObsTerm(func=my_obs_target_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # yaw_error = ObsTerm(func=my_obs_yaw_error, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # target_yaw_rate = ObsTerm(func=my_obs_target_yaw_rate, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # roll = ObsTerm(func=my_obs_roll, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # pitch = ObsTerm(func=my_obs_pitch, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # yaw = ObsTerm(func=my_obs_yaw, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        # La nueva función del target también necesita saber respecto a qué dron rotar
        # height = ObsTerm(func=my_obs_height, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
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
            self.history_length = 3 
            self.flatten_history_dim = True

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
        self._marker_visualizer_green = VisualizationMarkers(GREEN_ARROW_CFG)
        self._marker_visualizer_red = VisualizationMarkers(RED_ARROW_CFG)
        self._asset = env.scene[cfg.asset_name]
        
        # ponemos la velocidad y giro al que deberá ir nuestro punto guía
        # velocidad en x,y,z (tamaño 3)
        self.target_vel = torch.zeros(self.num_envs, 3, device=self.device)
        self.target_yaw = torch.zeros(self.num_envs,device=self.device)
        self.target_yaw_prev = torch.zeros(self.num_envs, device=self.device)

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

    def get_arrow_quat(self, direction_vec):
        """Convierte un vector de dirección en un cuaternión que apunta hacia esa dirección."""
        # 1. Normalizamos la dirección (Eje X de la flecha)
        x_axis = torch.nn.functional.normalize(direction_vec, dim=-1)
        
        # 2. Creamos una base ortonormal (Gram-Schmidt)
        # Usamos un vector 'up' auxiliar
        up = torch.tensor([0.0, 0.0, 1.0], device=self.device).repeat(direction_vec.shape[0], 1)
        # Eje Y = Up x X
        y_axis = torch.nn.functional.normalize(torch.cross(up, x_axis, dim=-1), dim=-1)
        # Eje Z = X x Y
        z_axis = torch.cross(x_axis, y_axis, dim=-1)
        
        # 3. Formamos la matriz de rotación [N, 3, 3] y convertimos a cuaternión
        # Isaac Lab espera las columnas en orden X, Y, Z
        res_matrix = torch.stack([x_axis, y_axis, z_axis], dim=-1)
        return quat_from_matrix(res_matrix)

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

        # Definimos velocidad aleatoria del fantasma (3 a 6 m/s)
        speed = torch.rand(len(env_ids), device=self.device) * 3.0 + 3.0
        
        # Dirección inicial aleatoria en el plano XY
        angle = torch.rand(len(env_ids), device=self.device) * 2 * math.pi
        self.target_vel[env_ids, 0] = torch.cos(angle) * speed
        self.target_vel[env_ids, 1] = torch.sin(angle) * speed
        self.target_vel[env_ids, 2] = torch.rand(len(env_ids), device=self.device) * 0.5 + 0.2
        # Curvatura aleatoria (Yaw rate del camino: -0.3 a 0.3 rad/s)
        # Esto genera círculos, curvas en S o rectas aleatorias
        max_yaw_rate_change = 0.06
        self.target_yaw[env_ids] = (torch.rand(len(env_ids), device=self.device) - 0.5) * 2.0 * max_yaw_rate_change
        self.target_yaw_prev[env_ids] = self.target_yaw[env_ids].clone()

    def _update_command(self):
        # segundo o más frame de tiempo
        """Mueve el fantasma en cada paso de física."""
        # Variación de altura dinámica durante el vuelo (cada ~2 seg)
        change_z = torch.rand(self.num_envs, device=self.device) < 0.01
        if change_z.any():
            self.target_vel[change_z, 2] = (torch.rand(change_z.sum(), device=self.device) - 0.5) * 1.6 # -0.8 a 0.8 m/s de variación de velocidad 

        # Limitar cambio de yaw del punto (aceleración angular)
        max_yaw_change = 0.02 * self.dt
        self.target_yaw = self.target_yaw_prev + torch.clamp(
            self.target_yaw - self.target_yaw_prev, -max_yaw_change, max_yaw_change
        )
        self.target_yaw_prev = self.target_yaw.clone()


        # Rotación horizontal (curvas)
        cos_theta = torch.cos(self.target_yaw * self.dt)
        sin_theta = torch.sin(self.target_yaw * self.dt)
        vx, vy = self.target_vel[:, 0].clone(), self.target_vel[:, 1].clone()
        self.target_vel[:, 0] = vx * cos_theta - vy * sin_theta
        self.target_vel[:, 1] = vx * sin_theta + vy * cos_theta

        # Valla Virtual (Límite 100m spacing -> 50m radio)
        limite_xy = 45.0
        limite_z = (1.75, 20.0)
        
        # # Rebote XY
        margin = 3.0

        out_x = (self.target_pos[:,0].abs() > limite_xy - margin)
        self.target_vel[out_x,0] *= -1.0

        out_y = (self.target_pos[:,1].abs() > limite_xy - margin)
        self.target_vel[out_y,1] *= -1.0

        # limite_xy = 45.0
        # margin = 6.0

        # pos_xy = self.target_pos[:, :2]

        # # distancia al borde
        # dist_to_edge = limite_xy - pos_xy.abs()

        # # factor de empuje (solo cerca del borde)
        # push_strength = torch.clamp((margin - dist_to_edge) / margin, min=0.0)

        # # dirección hacia el centro
        # push_dir = -torch.sign(pos_xy)

        # # fuerza suave
        # push = push_dir * push_strength * 0.5

        # self.target_vel[:, :2] += push

        # self.target_pos[:,0] = torch.clamp(self.target_pos[:,0], -limite_xy, limite_xy)
        # self.target_pos[:,1] = torch.clamp(self.target_pos[:,1], -limite_xy, limite_xy)

        # Z
        at_top = (self.target_pos[:,2] > limite_z[1]) & (self.target_vel[:,2] > 0)
        at_bot = (self.target_pos[:,2] < limite_z[0]) & (self.target_vel[:,2] < 0)
        self.target_vel[at_top | at_bot,2] *= -1.0

        # limitación de velocidad máxima
        max_speed = 6.0  # m/s
        speed = torch.norm(self.target_vel[:, :2], dim=1, keepdim=True)
        scale = torch.clamp(max_speed / (speed + 1e-6), max=1.0)
        self.target_vel[:, :2] *= scale

        # Actualizar posición del punto guía
        self.target_pos += self.target_vel * self.dt

        # Comando para la red (Posición relativa en Body Frame)
        uav_pos_local = self._asset.data.root_com_pos_w[:, :3] - self._env.scene.env_origins[:, :3]
        rel_pos_w = self.target_pos - uav_pos_local

        # Calcular yaw deseado hacia el objetivo
        delta = self.target_vel[:, :2] # ya no es target_pos - drone_pos
        target_yaw_angle = torch.atan2(delta[:, 1], delta[:, 0])
        current_yaw = math_utils.euler_xyz_from_quat(self._asset.data.root_com_quat_w)[2]
        yaw_error = target_yaw_angle - current_yaw
        yaw_error = (yaw_error + math.pi) % (2*math.pi) - math.pi  # [-pi, pi]


        quat_inv = math_utils.quat_inv(self._asset.data.root_com_quat_w)
        self._command[:, :3] = math_utils.quat_apply(quat_inv, rel_pos_w)
        
        # Guardamos la velocidad de giro del fantasma en la 4ª columna
        # Esto es lo que la recompensa rew_ang_vel_z_diff está buscando
        self._command[:, 3] = self.target_yaw 
        # ---------------------------
        # Visualización dinámica
        target_pos_w = self.target_pos + self._env.scene.env_origins
        self._marker_visualizer.visualize(translations=target_pos_w)

        # --- Flecha orientación objetivo (verde)
        delta_xy = self.target_pos[:, :2] - (self._asset.data.root_com_pos_w[:, :2] - self._env.scene.env_origins[:, :2])
        unit_vec = delta_xy / (torch.norm(delta_xy, dim=1, keepdim=True) + 1e-6)
        arrow_length = 2.0  # m
        arrow_vec = torch.zeros(self.num_envs, 3, device=self.device)
        arrow_vec[:, 0] = unit_vec[:, 0] * arrow_length
        arrow_vec[:, 1] = unit_vec[:, 1] * arrow_length
        arrow_vec[:, 2] = 0.1
        origin_pos_w = self._asset.data.root_com_pos_w
        # 1. Flecha Objetivo (Verde)
        # Vector desde el dron al target
        target_dir_w = target_pos_w - origin_pos_w
        quat_target = self.get_arrow_quat(target_dir_w)
        
        # 2. Flecha Dron (Roja)


        # --- VISUALIZAR ---

        # Si usas el mismo y quieres ver AMBAS, concatena:
        uav_pos_w = self._asset.data.root_com_pos_w[:, :3]
        target_pos_w = self.target_pos + self._env.scene.env_origins

        # 2. FLECHA OBJETIVO (Verde - Dirección hacia donde tiene que ir)
        target_dir_w = target_pos_w - uav_pos_w
        quat_target = self.get_arrow_quat(target_dir_w)
        
        # 3. FLECHA DRON (Roja - Hacia donde mira el morro del dron)
        # El forward del dron es su propio quaternion (si el asset mira hacia +X)
        quat_drone = self._asset.data.root_com_quat_w
        
        self._marker_visualizer_red.visualize(
            translations=uav_pos_w + torch.tensor([0, 0, 2.0], device=self.device), 
            orientations=quat_target,
            scales=torch.tensor([10.0, 3.0, 3.0], device=self.device).repeat(self.num_envs, 1)
        )

        # Flecha Roja (Orientación actual)
        self._marker_visualizer_green.visualize(
            translations=uav_pos_w + torch.tensor([0, 0, 1.0], device=self.device),
            orientations=quat_drone,
            scales=torch.tensor([8.0, 3.0, 3.0], device=self.device).repeat(self.num_envs, 1)
        )
        

@configclass
class UAVcommandTermCfg(CommandTermCfg):
    """Command term configuration for the UAV."""

    class_type: type = UAVcommandTerm
    """Class type of the command term."""
    asset_name: str = "aerotaxi"

@configclass
class CommandCfg:
    """Command specifications for the environment."""
    
    vel_command = UAVcommandTermCfg(asset_name="aerotaxi",resampling_time_range=(25, 25)) # cada 15 segundos cambiamos


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

    # Masa: 0.9 su masa y 1.1 su masa (con pasajeros, o por si alguno es especialmente menos pesado)
    randomize_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg(name="aerotaxi"),
            "mass_distribution_params": (0.9, 1.1), 
            "operation": "scale"
        }
    )
    # Viento: el dron recibirá una corriente de aire en diversas direcciones de forma aleatoria.
    randomize_wind = EventTerm(
        func=mdp.push_by_setting_velocity,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg(name="aerotaxi"),
            "velocity_range": {
                "x": (-1.0, 1.0), 
                "y": (-1.0, 1.0),
                "z": (-0.5, 0.5)
            }
        }
    )


# |---------------------------------------------------------|
# |--------------------- REWARDS ---------------------------|
# |---------------------------------------------------------|

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""
    rew_attitude_stability2 = RewTerm(func=my_rewards.rew_attitude_stability3, weight=2.0)
    rew_ang_vel_stability4 = RewTerm(func=my_rewards.rew_ang_vel_stability5, weight=1.0)
    rew_vel2 = RewTerm(func=my_rewards.rew_track_vel,weight=10.0)
    rew_pos2 = RewTerm(func=my_rewards.rew_track_pos, weight=3.0)
    rew_action_rate = RewTerm(func=my_rewards.rew_action_rate, weight=0.8)
    rew_heading = RewTerm(func=my_rewards.rew_heading, weight=4.0)

    # rew_vel_z = RewTerm(func=my_rewards.rew_vertical_velocity,weight=6.0)
    # tilt_penalty_pg = RewTerm(func=my_rewards.rew_tilt_penalty_pg,weight=-3.5)
    # rew_altitude_hold2 = RewTerm(func=my_rewards.rew_altitude_hold2,weight=4.0)

    # alive = RewTerm(func=mdp.is_alive, weight=2.0)
    # action_rate = RewTerm(func=my_rewards.rew_action_rate, weight=-0.01)
    # # rew_pos_fine = RewTerm(func=my_rewards.rew_pos_fine, weight=0.5)
    # rew_hover_stability = RewTerm(func=my_rewards.rew_hover_stability, weight=3.0)
    # rew_pos_diff_cuad = RewTerm(func=my_rewards.rew_pos_diff_cuad,weight=-1)
    # # rew_vel = RewTerm(func=my_rewards.rew_vel,weight=-0.4)
    # rew_ang_vel = RewTerm(func=my_rewards.rew_ang_vel,weight=-1.2)
    # # rew_attitude_stability = RewTerm(func=my_rewards.rew_attitude_stability,weight=4.0)
    # rew_altitude_hold = RewTerm(func=my_rewards.rew_altitude_hold,weight=3.0)
    # tilt_penalty_pg = RewTerm(func=my_rewards.rew_tilt_penalty_pg,weight=-3.5)





    # action_rate2 = RewTerm(func=my_rewards.rew_action_rate2, weight=-0.001)

    # terminating = RewTerm(func=mdp.is_terminated, weight=-100.0)

    # rew_pos_diff_xy = RewTerm(func=my_rewards.rew_pos_diff_xy, weight=6.0)
    # rew_pos_diffz = RewTerm(func=my_rewards.rew_pos_diff_z, weight=3.0)
    # rew_vel_z = RewTerm(func=my_rewards.rew_height_world_vel, weight=-4.0)
    # rew_emergency_climb = RewTerm(func=my_rewards.rew_emergency_climb, weight=2.0)
    # rew_pos_diff_exp = RewTerm(
    #     func=my_rewards.rew_pos_diff_exp,
    #     weight=5.0,
    # )

    

    # rew_vel_z = RewTerm(
    #     func=my_rewards.rew_vel_z,
    #     weight=-0.5
    # )

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
    # tilt_penalty = RewTerm(
    #     func=my_rewards.rew_tilt_penalty,
    #     weight=-0.05,
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
        params={"min_altitude": 0.05,} # el centro de masas del dron está a 5.06m del suelo.
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

GREEN_ARROW_CFG = VisualizationMarkersCfg(
    prim_path="/Visuals/TargetArrow",
    markers={
        "arrow": sim_utils.ConeCfg(
            radius=0.1,
            height=0.5,
            axis='X',
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)), # VERDE
        ),
    },
)


RED_ARROW_CFG = VisualizationMarkersCfg(
    prim_path="/Visuals/DroneArrow",
    markers={
        "arrow": sim_utils.ConeCfg(
            radius=0.1,
            height=0.5,
            axis='X',
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)), # ROJO
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
        