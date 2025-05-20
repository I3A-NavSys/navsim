from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
import isaaclab.utils.math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

def roll_pitch_termination(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Terminate when the asset's roll or pitch exceeds a certain threshold."""
    roll = env.obs_buf["policy"][:, 6]
    pitch = env.obs_buf["policy"][:, 7]
    
    return torch.logical_or(torch.abs(roll[:]) > torch.pi/3, torch.abs(pitch[:]) > torch.pi/3)

def lin_vel_z_termination(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Terminate when the asset's linear velocity in the z direction exceeds a certain threshold."""
    lin_vel_z = env.obs_buf["policy"][:, 2]
    
    return lin_vel_z[:] <= -50.0

def below_min_altitude(env: ManagerBasedRLEnv, min_altitude: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Terminate when the asset's altitude is below a certain threshold."""
    asset: Articulation = env.scene[asset_cfg.name]
    pos = asset.data.root_pos_w

    return pos[:, 2] <= min_altitude

def are_nan_values(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Terminate when obs buffer has any nan value """
    return torch.isnan(env.obs_buf["policy"][:]).any(dim=1)