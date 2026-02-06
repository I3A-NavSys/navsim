from __future__ import annotations

import torch
from typing import TYPE_CHECKING
from .flight_plan import FlightPlan
import isaaclab.utils.math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
def rew_action_rate(env: ManagerBasedRLEnv) -> torch.Tensor:
    # Penaliza la diferencia entre la acción actual y la anterior
    diff = torch.norm(env.action_manager.action - env.action_manager.prev_action, dim=1)
    return torch.clamp(diff, max=5.0)

def rew_pos_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    term = env.command_manager.get_term("vel_command")
    target_pos = term.target_pos 

    current_pos_w = env.scene["aerotaxi"].data.root_com_pos_w
    current_pos_local = current_pos_w[:, :3] - env.scene.env_origins[:, :3]
    
    error = torch.norm(current_pos_local - target_pos, dim=1)
    return torch.clamp(error, max=100.0)

def rew_pos_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
    term = env.command_manager.get_term("vel_command")
    target_pos = term.target_pos
    
    current_pos_w = env.scene["aerotaxi"].data.root_com_pos_w
    current_pos_local = current_pos_w[:, :3] - env.scene.env_origins[:, :3]
    
    distance = torch.norm(current_pos_local - target_pos, dim=1)
    return 1.0 - torch.tanh(distance / std)


def rew_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    vel_real = asset.data.root_com_lin_vel_b 
    
    # Obtenemos la velocidad del "fantasma"
    command_term = env.command_manager.get_term("vel_command")
    vel_plan_g = command_term.target_vel # World Frame
    
    # Transformamos plan a Body Frame
    quat_inv = math_utils.quat_inv(asset.data.root_com_quat_w)
    vel_plan_b = math_utils.quat_apply(quat_inv, vel_plan_g)

    # Error Euclidiano: sqrt((vx-vpx)^2 + (vy-vpy)^2 + (vz-vpz)^2)
    error = torch.norm(vel_real - vel_plan_b, dim=1)
    # if "metrics" not in env.extras: 
    #     env.extras["metrics"] = {}
    # env.extras["metrics"]["flight_plan_error_mps"] = error.mean()
    return torch.clamp(error, max=50.0)

def rew_lin_vel_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    lin_vel_b = asset.data.root_com_lin_vel_b
    vel_command = env.command_manager.get_command("vel_command")[:, :3]
    distance = torch.norm(lin_vel_b - vel_command, dim=1)
    return 1.0 - torch.tanh(distance / std)


def rew_ang_vel_z_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    ang_vel_z = asset.data.root_com_ang_vel_b[:, 2]
    yaw_rate_command = env.command_manager.get_command("vel_command")[:, 3]
    error = torch.abs(ang_vel_z - yaw_rate_command)
    return torch.clamp(error, max=5.0)

def rew_ang_vel_z_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    ang_vel_z = asset.data.root_com_ang_vel_b[:, 2]
    yaw_rate_command = env.command_manager.get_command("vel_command")[:, 3]
    distance = torch.abs(ang_vel_z - yaw_rate_command)
    return 1.0 - torch.tanh(distance / std)


def rew_roll_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    roll, _, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
    error = torch.abs(roll - target)
    return error


def rew_roll_diff_fine_grained(env: ManagerBasedRLEnv, target: float, std: float) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    roll, _, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
    distance = torch.abs(roll - target)
    return 1 - torch.tanh(distance/std)


def rew_pitch_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    _, pitch, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
    error = torch.abs(pitch - target)
    return error


def rew_pitch_diff_fine_grained(env: ManagerBasedRLEnv, target: float, std: float) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    _, pitch, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
    distance = torch.abs(pitch - target)
    return 1 - torch.tanh(distance/std)

# Recompensa de que el dron mire hacia el punto objetivo
def rew_heading_alignment_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
    term = env.command_manager.get_term("vel_command")
    target_pos = term.target_pos # (num_envs, 3)
    asset = env.scene["aerotaxi"]

    uav_pos_w = asset.data.root_com_pos_w[:, :3] - env.scene.env_origins[:, :3]
    target_vec_w = target_pos - uav_pos_w
    target_dir_w = target_vec_w / (torch.norm(target_vec_w, dim=1, keepdim=True) + 1e-6)

    quat_w = asset.data.root_com_quat_w
    # El vector [1, 0, 0] representa el "frente" del dron
    forward_b = torch.tensor([1.0, 0.0, 0.0], device=env.device).repeat(env.num_envs, 1)
    forward_w = math_utils.quat_apply(quat_w, forward_b)

    # 3. Producto escalar: 1.0 si apunta directo, -1.0 si está de espaldas
    # Solo nos interesa la alineación en el plano XY (navegación)
    dot_prod = torch.sum(forward_w[:, :2] * target_dir_w[:, :2], dim=1)
    
    # Error: qué tan lejos está de 1.0, que es que está mirando al objetivo
    error = 1.0 - dot_prod
    return 1.0 - torch.tanh(error / std)

def rew_ang_vel_xy_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    penalty = -torch.sum(torch.square(asset.data.root_com_ang_vel_b[:, :2]), dim=1)
    return torch.clamp(penalty, min=-50.0)
