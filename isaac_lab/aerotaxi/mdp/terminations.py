from __future__ import annotations

from scipy.spatial.transform import Rotation as R
import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

def roll_pitch_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Terminate when the asset's roll or pitch exceeds a certain threshold."""
    asset: Articulation = env.scene[asset_cfg.name]
    quat = asset.data.root_com_quat_w.tolist()
    quat = [quat[1], quat[2], quat[3], quat[0]]
    roll, pitch, yaw = R.from_quat(quat).as_euler('xyz')
    torch.pi
    
    return torch.logical_or(torch.abs(roll) > torch.pi/2, torch.abs(pitch) > torch.pi/2)

def lin_vel_z_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Terminate when the asset's linear velocity in the z direction exceeds a certain threshold."""
    asset: Articulation = env.scene[asset_cfg.name]
    lin_vel = asset.data.root_lin_vel_w
    
    return lin_vel[:, 2] <= -10.0