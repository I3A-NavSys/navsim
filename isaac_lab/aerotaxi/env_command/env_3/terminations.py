from __future__ import annotations

import torch
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

def roll_pitch_termination(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Terminate when the asset's roll or pitch exceeds a certain threshold."""
    roll = env.obs_buf["policy"][:, 9]
    pitch = env.obs_buf["policy"][:, 10]
    
    return torch.logical_or(torch.abs(roll[:]) > torch.pi/3, torch.abs(pitch[:]) > torch.pi/3)

def below_min_altitude(env: ManagerBasedRLEnv, min_altitude: float) -> torch.Tensor:
    """Terminate when the asset's altitude is below a certain threshold."""
    pos = env.obs_buf["policy"][:, 2]

    return pos <= min_altitude

def are_nan_values(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Terminate when obs buffer has any nan value """
    return torch.isnan(env.obs_buf["policy"][:]).any(dim=1)