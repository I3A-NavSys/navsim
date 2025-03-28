from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def lin_vel_diff(env: ManagerBasedRLEnv, target: torch.Tensor, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize linear velocity deviation from a target value."""
    asset: Articulation = env.scene[asset_cfg.name]
    lin_vel = asset.data.root_com_lin_vel_b
    diff = (lin_vel - target).norm(dim=1, keepdim=True)

    return diff