from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
import isaaclab.utils.math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

def roll_pitch_termination(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg = SceneEntityCfg(name="aerotaxi")) -> torch.Tensor:
    """Terminate when the asset's roll or pitch exceeds a certain threshold."""
    # roll = env.obs_buf["policy"][:, 9]
    # pitch = env.obs_buf["policy"][:, 10]
    
    # return torch.logical_or(torch.abs(roll[:]) > torch.pi/3, torch.abs(pitch[:]) > torch.pi/3)
    asset: Articulation = env.scene[asset_cfg.name]
    
    # Obtenemos la orientación (quaternions) desde los datos de la raíz
    quat_w = asset.data.root_com_quat_w
    
    # Convertimos a Euler (Roll, Pitch, Yaw) usando la utilidad de Isaac Lab
    roll, pitch, _ = math_utils.euler_xyz_from_quat(quat_w)
    
    # Aplicamos la lógica de terminación (60 grados = pi/3)
    limit = torch.pi / 3
    return torch.logical_or(torch.abs(roll) > limit, torch.abs(pitch) > limit)

def below_min_altitude(env: ManagerBasedRLEnv, min_altitude: float) -> torch.Tensor:
    """Terminate when the asset's altitude is below a certain threshold."""
    asset = env.scene["aerotaxi"]
    # print(asset.data.root_com_pos_w[:, 2])
    # root_com_pos_w[:, 2] es la coordenada Z global
    return asset.data.root_com_pos_w[:, 2] < min_altitude + 5.06 # el centro de masa del dron está a 5.06 metros de alto

def are_nan_or_exploded(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Terminate when obs buffer has any nan value """
    asset = env.scene["aerotaxi"]
    pos_w = asset.data.root_com_pos_w
    lin_vel = asset.data.root_com_lin_vel_w

    # 1. Detectar NaNs
    is_nan = torch.any(torch.isnan(pos_w), dim=1)
    
    # 2. Detectar explosión por posición (si sale volando a kilómetros de distancia)
    # Si el dron supera los 200m de altura o 200m de radio, algo va mal
    pos_local = pos_w - env.scene.env_origins
    is_too_far = torch.norm(pos_local[:, :2], dim=1) > 200.0
    is_too_high = pos_local[:, 2] > 200.0
    
    # 3. Detectar explosión por velocidad
    is_too_fast = torch.norm(lin_vel, dim=1) > 100.0

    return is_nan | is_too_far | is_too_high | is_too_fast