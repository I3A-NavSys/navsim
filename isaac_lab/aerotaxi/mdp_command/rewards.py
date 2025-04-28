from __future__ import annotations

import torch
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, :3]
    vel_command = obs["policy"][:, 8:12]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_10 = torch.tensor(10, device=env.device)
    torch_2 = torch.tensor(2, device=env.device)

    diff = lin_vel[:] - vel_command[:, :3]
    # diff_norm = torch.sqrt(torch.clamp(torch.sum(diff * diff, dim=1), min=1e-8))
    # diff_norm = diff.mean(dim=1)
    diff_norm = torch.norm(diff, dim=1)
    rewards[:] = torch_10 / torch_2 ** diff_norm

    return rewards

def ang_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    ang_vel_z = obs["policy"][:, 5]
    vel_command = obs["policy"][:, 8:12]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_10 = torch.tensor(10, device=env.device)
    torch_2 = torch.tensor(2, device=env.device)

    diff = (ang_vel_z[:] - vel_command[:, 3]).abs()
    rewards[:] = torch_10 / torch_2 ** diff

    return rewards

def jerky_mov(env: ManagerBasedRLEnv) -> torch.Tensor:
    current_actions = env.action_manager.action.abs()
    prev_actions = env.action_manager.prev_action.abs()
    # rewards = torch.zeros(env.num_envs, device=env.device)
    # torch_1_1 = torch.tensor(1.1, device=env.device)
    # torch_1 = torch.tensor(1, device=env.device)

    diff = current_actions - prev_actions
    # diff_norm = torch.sqrt(torch.clamp(torch.sum(diff * diff, dim=1), min=1e-8))
    # diff_norm = diff.mean(dim=1)
    diff_norm = torch.norm(diff, dim=1)
    # rewards[:] = (torch_1_1 ** diff_norm) - torch_1
    # rewards = torch.clamp(rewards, max=10)

    return diff_norm