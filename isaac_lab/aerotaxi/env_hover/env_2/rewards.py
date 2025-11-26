from __future__ import annotations

import torch
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def rew_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 3:6]
    vel_command = obs["policy"][:, 12:15]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_10 = torch.tensor(10, device=env.device)
    torch_2 = torch.tensor(2, device=env.device)

    diff = lin_vel[:] - vel_command[:]
    diff_norm = torch.norm(diff, dim=1)
    rewards[:] = torch_10 / (torch_2 ** diff_norm)

    return rewards


def rew_x_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 3]
    vel_command = obs["policy"][:, 12]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_3 = torch.tensor(3, device=env.device)
    torch_1 = torch.tensor(1, device=env.device)

    diff = (lin_vel - vel_command).abs()
    rewards[:] = torch_3 / (torch_1 + torch.exp(torch.log(diff)))

    return rewards


def rew_y_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 4]
    vel_command = obs["policy"][:, 13]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_3 = torch.tensor(3, device=env.device)
    torch_1 = torch.tensor(1, device=env.device)

    diff = (lin_vel - vel_command).abs()
    rewards[:] = torch_3 / (torch_1 + torch.exp(torch.log(diff)))

    return rewards


def rew_z_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 5]
    vel_command = obs["policy"][:, 14]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_3 = torch.tensor(3, device=env.device)
    torch_1 = torch.tensor(1, device=env.device)

    diff = (lin_vel - vel_command).abs()
    rewards[:] = torch_3 / (torch_1 + torch.exp(torch.log(diff)))

    return rewards


def rew_z_ang_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    ang_vel_z = obs["policy"][:, 8]
    vel_command = obs["policy"][:, 15]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_10 = torch.tensor(10, device=env.device)
    torch_1 = torch.tensor(1, device=env.device)

    diff = (ang_vel_z - vel_command).abs()
    rewards[:] = torch_10 / (torch_1 + torch.exp(torch.log(diff)))

    return rewards


def pen_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 3:6]
    vel_command = obs["policy"][:, 12:15]

    diff = lin_vel[:] - vel_command[:]
    diff_norm = torch.norm(diff, dim=1)

    return diff_norm


def pen_x_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 3]
    vel_command = obs["policy"][:, 12]

    diff = (lin_vel - vel_command).abs()

    return diff


def pen_y_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 4]
    vel_command = obs["policy"][:, 13]

    diff = (lin_vel - vel_command).abs()

    return diff


def pen_z_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 5]
    vel_command = obs["policy"][:, 14]

    diff = (lin_vel - vel_command).abs()

    return diff


def pen_z_ang_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    ang_vel_z = obs["policy"][:, 8]
    vel_command = obs["policy"][:, 15]

    diff = (ang_vel_z[:] - vel_command[:]).abs()

    return diff


def pen_roll_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    roll = env.obs_buf["policy"][:, 9]
    target = torch.tensor(target, device=env.device)

    diff = (roll[:] - target).abs()
    return diff


def pen_pitch_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    pitch = env.obs_buf["policy"][:, 10]
    target = torch.tensor(target, device=env.device)

    diff = (pitch[:] - target).abs()

    return diff


def pen_roll_excess(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    """Penalize roll excess from target limit"""
    roll = env.obs_buf["policy"][:, 9]
    rewards = torch.zeros(env.num_envs, device=env.device)
    target = torch.tensor(target, device=env.device)
    
    excess_mask = roll[:] > target
    diff = (roll[:] - target).abs()
    rewards[excess_mask] = diff[excess_mask]
    
    return rewards


def pen_pitch_excess(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    """Penalize roll excess from target limit"""
    pitch = env.obs_buf["policy"][:, 10]
    rewards = torch.zeros(env.num_envs, device=env.device)
    target = torch.tensor(target, device=env.device)
    
    excess_mask = pitch[:] > target
    diff = (pitch[:] - target).abs()
    rewards[excess_mask] = diff[excess_mask]
    
    return rewards