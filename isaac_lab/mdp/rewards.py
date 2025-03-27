from __future__ import annotations

from scipy.spatial.transform import Rotation as R
import numpy as np
import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils.math import wrap_to_pi
import numpy as np

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def lin_vel_diff(env: ManagerBasedRLEnv, target: np.array, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize linear velocity deviation from a target value."""
    asset: Articulation = env.scene[asset_cfg.name]
    lin_vel = np.array(asset.data.root_com_lin_vel_b.tolist())
    
    # compute the reward
    return torch.tensor(np.linalg.norm(lin_vel - target), device=env.device)

def roll_pitch_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Terminate when the asset's roll or pitch exceeds a certain threshold."""
    asset: Articulation = env.scene[asset_cfg.name]
    quat = asset.data.root_com_quat_w.tolist()
    quat = [quat[1], quat[2], quat[3], quat[0]]
    roll, pitch, yaw = R.from_quat(quat).as_euler('xyz')
    
    return torch.logical_or(torch.abs(roll) > np.pi/2, torch.abs(pitch) > np.pi/2)