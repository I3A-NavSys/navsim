from __future__ import annotations

import torch
from typing import TYPE_CHECKING
from .flight_plan import FlightPlan
import isaaclab.utils.math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
def rew_action_rate(env: ManagerBasedRLEnv) -> torch.Tensor:
    # Penaliza la diferencia entre la acción actual y la anterior
    return torch.norm(env.action_manager.action - env.action_manager.prev_action, dim=1)

def rew_pos_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    term = env.command_manager.get_term("vel_command")
    target_pos = term.target_pos 

    current_pos_w = env.scene["aerotaxi"].data.root_com_pos_w
    current_pos_local = current_pos_w - env.scene.env_origins
    
    error = torch.norm(current_pos_local - target_pos, dim=1)
    return torch.clamp(error, max=10.0)

def rew_pos_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
    term = env.command_manager.get_term("vel_command")
    target_pos = term.target_pos
    
    current_pos_w = env.scene["aerotaxi"].data.root_com_pos_w
    current_pos_local = current_pos_w - env.scene.env_origins
    
    distance = torch.norm(current_pos_local - target_pos, dim=1)
    return 1.0 - torch.tanh(distance / std)


def rew_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    lin_vel_b = asset.data.root_com_lin_vel_b
    vel_command = env.command_manager.get_command("vel_command")[:, :3]
    
    error = torch.norm(lin_vel_b - vel_command, dim=1)
    return torch.clamp(error, max=10.0)


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
    return torch.clamp(error, max=torch.pi)


def rew_roll_diff_fine_grained(env: ManagerBasedRLEnv, target: float, std: float) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    roll, _, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
    distance = torch.abs(roll - target)
    return 1 - torch.tanh(distance/std)


def rew_pitch_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    _, pitch, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
    error = torch.abs(pitch - target)
    return torch.clamp(error, max=torch.pi)


def rew_pitch_diff_fine_grained(env: ManagerBasedRLEnv, target: float, std: float) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    _, pitch, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
    distance = torch.abs(pitch - target)
    return 1 - torch.tanh(distance/std)
