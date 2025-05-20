from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
import isaaclab.utils.math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def rew_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize linear velocity deviation from a target value."""
    obs = env.obs_buf
    lin_vel = obs["policy"][:, :3]
    # vel_command = env.command_manager.get_command("vel_command")
    vel_command = obs["policy"][:, 8:12]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_10 = torch.tensor(10, device=env.device)
    torch_1 = torch.tensor(1, device=env.device)

    diff = lin_vel[:] - vel_command[:, :3]
    diff_norm = torch.sqrt(torch.clamp(torch.sum(diff * diff, dim=1), min=1e-8))
    rewards[:] = torch_10 / (torch_1 + torch.exp(torch.log(diff_norm)))

    return rewards

def rew_x_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 0]
    vel_command = obs["policy"][:, 8]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_3 = torch.tensor(3, device=env.device)
    torch_1 = torch.tensor(1, device=env.device)

    diff = (lin_vel - vel_command).abs()
    rewards[:] = torch_3 / (torch_1 + torch.exp(torch.log(diff)))

    return rewards

def rew_y_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 1]
    vel_command = obs["policy"][:, 9]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_3 = torch.tensor(3, device=env.device)
    torch_1 = torch.tensor(1, device=env.device)

    diff = (lin_vel - vel_command).abs()
    rewards[:] = torch_3 / (torch_1 + torch.exp(torch.log(diff)))

    return rewards

def rew_z_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 2]
    vel_command = obs["policy"][:, 10]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_3 = torch.tensor(3, device=env.device)
    torch_1 = torch.tensor(1, device=env.device)

    diff = (lin_vel - vel_command).abs()
    rewards[:] = torch_3 / (torch_1 + torch.exp(torch.log(diff)))

    return rewards

def rew_z_ang_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    ang_vel_z = obs["policy"][:, 5]
    vel_command = obs["policy"][:, 11]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_10 = torch.tensor(10, device=env.device)
    torch_1 = torch.tensor(1, device=env.device)

    diff = (ang_vel_z - vel_command).abs()
    rewards[:] = torch_10 / (torch_1 + torch.exp(torch.log(diff)))

    return rewards

def pen_jerky_mov(env: ManagerBasedRLEnv) -> torch.Tensor:
    current_actions = env.action_manager.action.abs()
    prev_actions = env.action_manager.prev_action.abs()

    diff = current_actions - prev_actions
    diff_norm = torch.norm(diff, dim=1)

    return diff_norm

def pen_roll_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    """Penalize roll and pitch deviation from a target value."""
    roll = env.obs_buf["policy"][:, 6]
    target = torch.tensor(target, device=env.device)

    diff = (roll[:] - target).abs()
    return diff

def pen_pitch_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    """Penalize roll and pitch deviation from a target value."""
    pitch = env.obs_buf["policy"][:, 7]
    target = torch.tensor(target, device=env.device)

    diff = (pitch[:] - target).abs()

    return diff