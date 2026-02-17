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

# def rew_pos_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
#     term = env.command_manager.get_term("vel_command")
#     target_pos = term.target_pos 

#     current_pos_w = env.scene["aerotaxi"].data.root_com_pos_w
#     current_pos_local = current_pos_w[:, :3] - env.scene.env_origins[:, :3]
    
#     error = torch.norm(current_pos_local - target_pos, dim=1)
#     return torch.clamp(error, max=100.0)

# def rew_pos_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
#     term = env.command_manager.get_term("vel_command")
#     target_pos = term.target_pos
    
#     current_pos_w = env.scene["aerotaxi"].data.root_com_pos_w
#     current_pos_local = current_pos_w[:, :3] - env.scene.env_origins[:, :3]
    
#     distance = torch.norm(current_pos_local - target_pos, dim=1)
#     return 1.0 - torch.tanh(distance / std)
def rew_pos_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    command_term = env.command_manager.get_term("vel_command")

    # Posición actual en world
    pos_w = asset.data.root_com_pos_w[:, :3]
    pos_local = pos_w - env.scene.env_origins[:, :3]

    distance = torch.norm(command_term.target_pos - pos_local, dim=1)
    return 1.0 - torch.tanh(distance / std)



# def rew_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
#     asset = env.scene["aerotaxi"]
#     lin_vel_b = asset.data.root_com_lin_vel_b
#     vel_command = env.command_manager.get_command("vel_command")[:, :3]
    
#     error = torch.norm(lin_vel_b - vel_command, dim=1)
#     return torch.clamp(error, max=10.0)

def rew_lin_vel_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    command_term = env.command_manager.get_term("vel_command")

    lin_vel_b = asset.data.root_com_lin_vel_b

    # Target velocity en world
    target_vel_w = command_term.target_vel

    # Convertimos target a body frame
    quat_inv = math_utils.quat_inv(asset.data.root_com_quat_w)
    target_vel_b = math_utils.quat_apply(quat_inv, target_vel_w)

    # pruebo a descomponer el error de velocidad en xy y en z, para dar más peso a la z
    vel_error_xy = torch.norm(lin_vel_b[:, :2] - target_vel_b[:, :2], dim=1)
    vel_error_z = torch.abs(lin_vel_b[:, 2] - target_vel_b[:, 2])
    combined_error = vel_error_xy + (2 * vel_error_z)
    return 1.0 - torch.tanh(combined_error / std)



# def rew_ang_vel_z_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
#     asset = env.scene["aerotaxi"]
#     ang_vel_z = asset.data.root_com_ang_vel_b[:, 2]
#     yaw_rate_command = env.command_manager.get_command("vel_command")[:, 3]
#     error = torch.abs(ang_vel_z - yaw_rate_command)
#     return torch.clamp(error, max=5.0)

def rew_ang_vel_z_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    command_term = env.command_manager.get_term("vel_command")

    ang_vel_z = asset.data.root_com_ang_vel_b[:, 2]
    target_yaw_rate = command_term.target_yaw

    error = torch.abs(ang_vel_z - target_yaw_rate)
    return 1.0 - torch.tanh(error / std)



# def rew_roll_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
#     asset = env.scene["aerotaxi"]
#     roll, _, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
#     error = torch.abs(roll - target)
#     return error


def rew_roll_diff_fine_grained(env: ManagerBasedRLEnv, target: float, std: float) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    roll, _, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
    distance = torch.abs(roll - target)
    return 1 - torch.tanh(distance/std)


# def rew_pitch_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
#     asset = env.scene["aerotaxi"]
#     _, pitch, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
#     error = torch.abs(pitch - target)
#     return error


def rew_pitch_diff_fine_grained(env: ManagerBasedRLEnv, target: float, std: float) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    _, pitch, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
    distance = torch.abs(pitch - target)
    return 1 - torch.tanh(distance/std)

# def rew_hovering(env: ManagerBasedRLEnv, min_altitude: float, max_altitude: float):
#     asset = env.scene["aerotaxi"]
#     z = asset.data.root_com_pos_w[:, 2] - env.scene.env_origins[:, 2]

#     inside = (z >= min_altitude) & (z <= max_altitude)
#     return inside.float()


def rew_ang_vel_xy_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
    asset = env.scene["aerotaxi"]
    ang_vel_xy = asset.data.root_com_ang_vel_b[:, :2]
    penalty = torch.sum(torch.square(ang_vel_xy), dim=1)
    return -torch.clamp(penalty, max=50.0)

