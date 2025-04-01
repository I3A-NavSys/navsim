from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
import isaaclab.utils.math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

def roll_pitch_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Terminate when the asset's roll or pitch exceeds a certain threshold."""
    asset: Articulation = env.scene[asset_cfg.name]
    roll, pitch, _ = math_utils.euler_xyz_from_quat(asset.data.root_quat_w)
    
    # # normalize angle to [-pi, pi]
    roll = torch.atan2(torch.sin(roll), torch.cos(roll))
    pitch = torch.atan2(torch.sin(pitch), torch.cos(pitch))
    
    return torch.logical_or(torch.abs(roll) > torch.pi/2, torch.abs(pitch) > torch.pi/2)

def lin_vel_z_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Terminate when the asset's linear velocity in the z direction exceeds a certain threshold."""
    asset: Articulation = env.scene[asset_cfg.name]
    lin_vel = asset.data.root_lin_vel_w
    
    return lin_vel[:, 2] <= -50.0

def below_min_altitude(env: ManagerBasedRLEnv, min_altitude: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Terminate when the asset's altitude is below a certain threshold."""
    asset: Articulation = env.scene[asset_cfg.name]
    pos = asset.data.root_pos_w

    return pos[:, 2] <= min_altitude