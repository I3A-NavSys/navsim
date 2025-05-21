# Standard library imports
from __future__ import annotations
from typing import TYPE_CHECKING

# Related third party imports
import torch

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize linear velocity deviation from zero."""
    observations = env.obs_buf["policy"]
    lin_vel = observations[:, :3]
    rewards = torch.zeros(env.num_envs, device=env.device)
    reference  = observations[:, 8:11]
    diff = lin_vel[:] - reference
    diff = torch.norm(diff, dim=1)
    rewards[:] = 10.0 / (1.0 + diff)
    return rewards

def ang_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize linear velocity deviation from zero."""
    observations = env.obs_buf["policy"]
    ang_vel = observations[:, 5]
    rewards = torch.zeros(env.num_envs, device=env.device)
    reference  = observations[:, 11]
    diff = ang_vel[:] - reference
    diff = torch.norm(diff)
    rewards[:] = 10.0 / (1.0 + diff)
    return rewards

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