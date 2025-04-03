from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
import isaaclab.utils.math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def lin_vel_diff(env: ManagerBasedRLEnv, target: list, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize linear velocity deviation from a target value."""
    asset: Articulation = env.scene[asset_cfg.name]
    lin_vel = asset.data.root_com_lin_vel_b
    target = torch.tensor(target, device=env.device)

    diff = lin_vel[:] - target
    diff_norm = diff.norm(dim=1, keepdim=True).flatten()
    diff_rewards = []
    for i in range(diff_norm.size(0)):
        value = diff_norm[i].log().abs()
        # if value == 1: value = 1.1
        # if value < 1: value = 1.1
        diff_rewards.append(3 / value)

    return torch.tensor(diff_rewards, device=env.device)

def lin_vel_z_diff(env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize linear velocity in the z direction deviation from a target value."""
    asset: Articulation = env.scene[asset_cfg.name]
    lin_vel_z = asset.data.root_com_lin_vel_b[:, 2]  # Z-axis velocity
    target = torch.tensor(target, device=env.device)

    diff = lin_vel_z[:].abs() - target

    return diff.flatten()

def modern_control_diff(env: ManagerBasedRLEnv, target: list, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize actions deviation from the modern control."""

    def servo_control(env_id: int) -> torch.Tensor:
        obs = env.obs_buf
        lin_vel = obs["policy"][env_id, 3:6]
        ang_vel = obs["policy"][env_id, 6:9]
        roll, pitch, _ = obs["policy"][env_id, 9:12]
        w_rotors = torch.tensor([0, 0, 0, 0], device=env.device)  # Rotor speed

        w_hover = 41.8879
        w_max = 62.8319
        w_min = 0
        
        r = torch.zeros(4, 1, device=env.device)  # Reference
        x = torch.zeros(8, 1, device=env.device)  # State
        y = torch.zeros(4, 1, device=env.device)  # Output
        e = torch.zeros(4, 1, device=env.device)  # Error
        E = torch.zeros(4, 1, device=env.device)  # Cumulative error
        u = torch.zeros(4, 1, device=env.device)  # Control signal

        Kx = torch.tensor([        # state control matrix
            [ -14.6551,  -45.5032,   -4.1872,  -13.0009,    3.8871,   -6.6331,    2.1363,    6.4114 ],
            [  14.6551,  -45.5032,    4.1872,  -13.0009,   -3.8871,   -6.6331,   -2.1363,    6.4114 ],
            [ -58.6206,  227.5161,  -16.7487,   65.0046,  -24.4514,   33.1656,    8.5453,    6.4114 ],
            [  58.6206,  227.5161,   16.7487,   65.0046,   24.4514,   33.1656,   -8.5453,    6.4114 ]
        ], device=env.device)

        Ky = torch.tensor([        # error control matrix
            [ -3.1839,    1.0254,    4.2743,    2.5914 ],
            [ -3.1839,   -1.0254,    4.2743,   -2.5914 ],
            [ 15.9195,    4.1017,    4.2743,  -16.3009 ],
            [ 15.9195,   -4.1017,    4.2743,   16.3009 ]           
        ], device=env.device)

        Hs = torch.tensor([        # hovering speed
            [w_hover], 
            [w_hover], 
            [w_hover], 
            [w_hover]
        ], device=env.device)

        # Assign the model reference to be followed
        r[0, 0] = target[0]       # bXdot
        r[1, 0] = target[1]       # bYdot
        r[2, 0] = target[2]       # bZdot
        r[3, 0] = target[3]       # hZdot

        # Assign model state
        x[0, 0] = roll           # ePhi
        x[1, 0] = pitch          # eTheta
        x[2, 0] = ang_vel[0] # bWx
        x[3, 0] = ang_vel[1] # bWy
        x[4, 0] = ang_vel[2] # bWz
        x[5, 0] = lin_vel[0]  # bXdot
        x[6, 0] = lin_vel[1]  # bYdot
        x[7, 0] = lin_vel[2]  # bZdot

        # Assign model output
        y[0, 0] = x[5, 0]        # bXdot
        y[1, 0] = x[6, 0]        # bYdot
        y[2, 0] = x[7, 0]        # bZdot
        y[3, 0] = x[4, 0]        # bWz

        # Error between the output and the reference 
        # (between the commanded velocity and the drone velocity)
        e = y - r

        # Cumulative error
        E = E + (e * env.step_dt)

        # Dynamic system control
        u = Hs - Kx @ x - Ky @ E

        # Rotor speed saturation
        if u[0, 0] > w_max : u[0, 0] = w_max
        if u[0, 0] < w_min : u[0, 0] = w_min
        if u[1, 0] > w_max : u[1, 0] = w_max
        if u[1, 0] < w_min : u[1, 0] = w_min
        if u[2, 0] > w_max : u[2, 0] = w_max
        if u[2, 0] < w_min : u[2, 0] = w_min
        if u[3, 0] > w_max : u[3, 0] = w_max
        if u[3, 0] < w_min : u[3, 0] = w_min

        w_rotors[0] = u[1, 0]   #NW
        w_rotors[1] = u[0, 0]   #NE
        w_rotors[2] = u[3, 0]   #SW
        w_rotors[3] = u[2, 0]   #SE

        return w_rotors
    
    # asset: Articulation = env.scene[asset_cfg.name]
    action_scale = 10
    diff = torch.zeros(env.num_envs, 1, device=env.device)  # Control signal

    for i, action in enumerate(env.action_manager.action):
        w_rotors = servo_control(i)
        scaled_action = action.abs() * action_scale
        # scaled_action = action.abs()

        sum = (w_rotors - scaled_action).abs().sum()
        if sum.item() == 1:    sum = torch.tensor(1.1, device=env.device)
        if sum.item() < 1:     sum = torch.tensor(1.1, device=env.device)
        diff[i, 0] = 3 / torch.log(sum)

    return diff.flatten()

def ang_vel_diff(env: ManagerBasedRLEnv, target: list, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize angular velocity deviation from a target value."""
    asset: Articulation = env.scene[asset_cfg.name]
    ang_vel = asset.data.root_com_ang_vel_b
    target = torch.tensor(target, device=env.device)

    diff = (ang_vel[:] - target)
    diff = diff.norm(dim=1, keepdim=True).flatten()

    return diff

def roll_diff(env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize roll and pitch deviation from a target value."""
    roll = env.obs_buf["policy"][:, 9]
    target = torch.tensor(target, device=env.device)

    diff = roll[:].abs() - target

    return diff

def pitch_diff(env: ManagerBasedRLEnv, target: float, asset_cfg: SceneEntityCfg) -> torch.Tensor:
    """Penalize roll and pitch deviation from a target value."""
    pitch = env.obs_buf["policy"][:, 10]
    target = torch.tensor(target, device=env.device)

    diff = pitch[:].abs() - target

    return diff