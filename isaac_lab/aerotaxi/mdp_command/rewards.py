from __future__ import annotations

import torch
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, :3]
    vel_command = obs["policy"][:, 8:11]    # Angular velocity is not needed
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_10 = torch.tensor(10, device=env.device)
    torch_2 = torch.tensor(2, device=env.device)

    diff = lin_vel[:] - vel_command[:]
    diff_norm = torch.norm(diff, dim=1)
    rewards[:] = torch_10 / torch_2 ** diff_norm

    return rewards

def ang_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    ang_vel_z = obs["policy"][:, 5]
    vel_command = obs["policy"][:, 11]  # Linear velocity is not needed
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_10 = torch.tensor(10, device=env.device)
    torch_2 = torch.tensor(2, device=env.device)

    diff = (ang_vel_z[:] - vel_command[:]).abs()
    rewards[:] = torch_10 / torch_2 ** diff

    return rewards

def jerky_mov(env: ManagerBasedRLEnv) -> torch.Tensor:
    current_actions = env.action_manager.action.abs()
    prev_actions = env.action_manager.prev_action.abs()

    diff = current_actions - prev_actions
    diff_norm = torch.norm(diff, dim=1)

    return diff_norm

def pen_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize linear velocity in the z direction deviation from a target value."""
    obs = env.obs_buf
    lin_vel = obs["policy"][:, :3]
    vel_command = obs["policy"][:, 8:11]    # Angular velocity is not needed
    rewards = torch.zeros(env.num_envs, device=env.device)

    is_hover_mask = torch.sum(vel_command, dim=1) == 0.0
    diff = lin_vel[:] - vel_command[:]
    diff_norm = torch.norm(diff, dim=1)
    rewards[is_hover_mask] = diff_norm[is_hover_mask]

    return rewards

def roll_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    """Penalize roll and pitch deviation from a target value."""
    obs = env.obs_buf
    roll = obs["policy"][:, 6]
    vel_command = obs["policy"][:, 8:11]    # Angular velocity is not needed
    rewards = torch.zeros(env.num_envs, device=env.device)
    target = torch.tensor(target, device=env.device)

    is_hover_mask = torch.sum(vel_command, dim=1) == 0.0
    diff = (roll[:] - target).abs()
    rewards[is_hover_mask] = diff[is_hover_mask]

    return rewards

def pitch_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    """Penalize roll and pitch deviation from a target value."""
    obs = env.obs_buf
    pitch = obs["policy"][:, 7]
    vel_command = obs["policy"][:, 8:11]    # Angular velocity is not needed
    rewards = torch.zeros(env.num_envs, device=env.device)
    target = torch.tensor(target, device=env.device)

    is_hover_mask = torch.sum(vel_command, dim=1) == 0.0
    diff = (pitch[:] - target).abs()
    rewards[is_hover_mask] = diff[is_hover_mask]

    return rewards