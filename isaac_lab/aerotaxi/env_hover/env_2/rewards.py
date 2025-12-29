from __future__ import annotations

import torch
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def rew_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 3:6]  
    vel_command = obs["policy"][:, 12:15]
    error = torch.norm(lin_vel - vel_command, dim=1)
    return torch.clamp(error, max=20.0)  # evito recompensa infinita con máximo 20 m/s


def rew_lin_vel_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 3:6]  
    vel_command = obs["policy"][:, 12:15]
    distance = torch.norm(lin_vel - vel_command, dim=1)
    return 1 - torch.tanh(distance/std)


def rew_ang_vel_z_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    ang_vel = obs["policy"][:, 8]  
    ang_vel_command = obs["policy"][:, 15]
    error = torch.abs(ang_vel - ang_vel_command)
    return torch.clamp(error, max=10.0) # maximo 10 rad/s


def rew_ang_vel_z_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
    obs = env.obs_buf
    ang_vel = obs["policy"][:, 8]  
    ang_vel_command = obs["policy"][:, 15]
    distance = torch.abs(ang_vel - ang_vel_command)
    return 1 - torch.tanh(distance/std)


def rew_roll_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    obs = env.obs_buf
    roll = obs["policy"][:, 9].squeeze()  
    roll_command = torch.tensor(target, device=env.device)
    error = torch.abs(roll - roll_command)
    return torch.clamp(error, max=torch.pi)


def rew_roll_diff_fine_grained(env: ManagerBasedRLEnv, target: float, std: float) -> torch.Tensor:
    obs = env.obs_buf
    roll = obs["policy"][:, 9].squeeze()  
    roll_command = torch.tensor(target, device=env.device)
    distance = torch.abs(roll - roll_command)
    return 1 - torch.tanh(distance/std)


def rew_pitch_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    obs = env.obs_buf
    pitch = obs["policy"][:, 10].squeeze()  
    pitch_command = torch.tensor(target, device=env.device)
    error = torch.abs(pitch - pitch_command)
    return torch.clamp(error, max=torch.pi)


def rew_pitch_diff_fine_grained(env: ManagerBasedRLEnv, target: float, std: float) -> torch.Tensor:
    obs = env.obs_buf
    pitch = obs["policy"][:, 10].squeeze() 
    pitch_command = torch.tensor(target, device=env.device)
    distance = torch.abs(pitch - pitch_command)
    return 1 - torch.tanh(distance/std)


