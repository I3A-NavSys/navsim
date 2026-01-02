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
    roll = env.obs_buf["policy"][:, 9]
    pitch = env.obs_buf["policy"][:, 10]
    
    return torch.logical_or(torch.abs(roll[:]) > torch.pi/3, torch.abs(pitch[:]) > torch.pi/3)

def below_min_altitude(env: ManagerBasedRLEnv, min_altitude: float) -> torch.Tensor:
    """Terminate when the asset's altitude is below a certain threshold."""
    pos = env.obs_buf["policy"][:, 2]
    min_altitude = torch.tensor(min_altitude, device=env.device)

    return pos <= min_altitude

def are_nan_or_exploded(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Terminate when obs buffer has any nan value """
    asset = env.scene["aerotaxi"]
    pos = asset.data.root_com_pos_w
    lin_vel = asset.data.root_com_lin_vel_w

    # 1. Detectar NaNs
    is_nan = torch.any(torch.isnan(pos), dim=1)
    
    # 2. Detectar explosión por posición (si sale volando a kilómetros de distancia)
    # Si el dron supera los 200m de altura o 200m de radio, algo va mal
    is_too_far = torch.norm(pos[:, :2], dim=1) > 200.0
    is_too_high = pos[:, 2] > 200.0
    
    # 3. Detectar explosión por velocidad
    is_too_fast = torch.norm(lin_vel, dim=1) > 100.0

    return is_nan | is_too_far | is_too_high | is_too_fast