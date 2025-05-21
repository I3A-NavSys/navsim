# Standard library imports
from __future__ import annotations
from typing import TYPE_CHECKING

# Related third party imports
import torch
from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import math

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

def roll(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    asset: Articulation = env.scene[asset_cfg.name]
    roll, _, _ = math.euler_xyz_from_quat(asset.data.root_quat_w)
    roll = torch.atan2(torch.sin(roll), torch.cos(roll))
    roll = roll.unsqueeze(1)
    return roll

def pitch(env:ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    asset: Articulation = env.scene[asset_cfg.name]
    _, pitch, _ = math.euler_xyz_from_quat(asset.data.root_quat_w)
    pitch = torch.atan2(torch.sin(pitch), torch.cos(pitch))
    pitch = pitch.unsqueeze(1)
    return pitch

def command(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Get current command."""
    return env.command_manager.get_command("command")