from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize linear velocity deviation from zero."""
    observations = env.obs_buf
    lin_vel = observations["policy"][:, :3]
    rewards = torch.zeros(env.num_envs, device=env.device)
    reference  = torch.zeros(env.num_envs, 3, device=env.device)
    diff = lin_vel[:] - reference
    diff = torch.norm(diff, dim=1)
    rewards[:] = 10.0 / (1.0 + diff)
    return rewards

def ang_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize linear velocity deviation from zero."""
    observations = env.obs_buf
    ang_vel = observations["policy"][:, 3:6]
    rewards = torch.zeros(env.num_envs, device=env.device)
    reference  = torch.zeros(env.num_envs, 3, device=env.device)
    diff = ang_vel[:] - reference
    diff = torch.norm(diff, dim=1)
    rewards[:] = 10.0 / (1.0 + diff)
    return rewards