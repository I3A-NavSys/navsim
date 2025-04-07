from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation
from isaaclab.managers import SceneEntityCfg
import isaaclab.utils.math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize linear velocity deviation from a target value."""
    obs = env.obs_buf
    lin_vel = obs["policy"][:, :3]
    vel_command = env.command_manager.get_command("vel_command")
    rewards = torch.zeros(env.num_envs, device=env.device)

    diff = (lin_vel[:] - vel_command[:, :3]).abs()
    diff = diff.norm(dim=1, keepdim=True).flatten()
    rewards[:] = 10 / (1 + torch.exp(torch.log(diff)))

    return rewards

def lin_vel_0(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, :3]
    vel_command = env.command_manager.get_command("vel_command")
    rewards = torch.zeros(env.num_envs, device=env.device)

    diff = lin_vel.abs().norm(dim=1, keepdim=True).flatten()
    flatten_vel = vel_command[:, :3].abs().norm(dim=1, keepdim=True).flatten()

    rewards[:] = (10 / (1 + torch.exp(torch.log(diff)))) * flatten_vel

    return rewards

def lin_vel_z_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize linear velocity in the z direction deviation from a target value."""
    obs = env.obs_buf
    lin_vel_z = obs["policy"][:, 2]
    vel_command = env.command_manager.get_command("vel_command")

    diff = (lin_vel_z[:] - vel_command[:, 2]).abs()

    return diff.flatten()

def modern_control_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize actions deviation from the modern control (vectorized version)."""
    
    # Constants
    w_hover = torch.tensor(41.8879, device=env.device)
    w_max = torch.tensor(62.8319, device=env.device)
    w_min = torch.tensor(0.0, device=env.device)
    action_scale = torch.tensor(10.0, device=env.device)
    
    # Control matrices
    Kx = torch.tensor([
        [-14.6551, -45.5032, -4.1872, -13.0009, 3.8871, -6.6331, 2.1363, 6.4114],
        [14.6551, -45.5032, 4.1872, -13.0009, -3.8871, -6.6331, -2.1363, 6.4114],
        [-58.6206, 227.5161, -16.7487, 65.0046, -24.4514, 33.1656, 8.5453, 6.4114],
        [58.6206, 227.5161, 16.7487, 65.0046, 24.4514, 33.1656, -8.5453, 6.4114]
    ], device=env.device)

    Ky = torch.tensor([
        [-3.1839, 1.0254, 4.2743, 2.5914],
        [-3.1839, -1.0254, 4.2743, -2.5914],
        [15.9195, 4.1017, 4.2743, -16.3009],
        [15.9195, -4.1017, 4.2743, 16.3009]
    ], device=env.device)

    Hs = torch.full((4, 1), w_hover, device=env.device)
    
    # Get observations for all environments
    obs = env.obs_buf["policy"]
    lin_vel = obs[:, :3]  # shape: (num_envs, 3)
    ang_vel = obs[:, 3:6]  # shape: (num_envs, 3)
    roll = env.obs_buf["policy"][:, 6] # shape: (num_envs, 1)
    pitch = env.obs_buf["policy"][:, 7] # shape: (num_envs, 1)
    vel_command = env.command_manager.get_command("vel_command")
    rewards = torch.zeros(env.num_envs, device=env.device)

    # Create reference tensor
    r = torch.zeros(env.num_envs, 4, 1, device=env.device)
    r[:, 0, 0] = vel_command[:, 0]  # bXdot
    r[:, 1, 0] = vel_command[:, 1]  # bYdot
    r[:, 2, 0] = vel_command[:, 2]  # bZdot
    r[:, 3, 0] = vel_command[:, 3]  # hZdot
    
    # Build state tensor x (num_envs, 8, 1)
    x = torch.zeros(env.num_envs, 8, 1, device=env.device)
    x[:, 0, 0] = roll  # roll
    x[:, 1, 0] = pitch  # pitch
    x[:, 2, 0] = ang_vel[:, 0]  # bWx
    x[:, 3, 0] = ang_vel[:, 1]  # bWy
    x[:, 4, 0] = ang_vel[:, 2]  # bWz
    x[:, 5, 0] = lin_vel[:, 0]  # bXdot
    x[:, 6, 0] = lin_vel[:, 1]  # bYdot
    x[:, 7, 0] = lin_vel[:, 2]  # bZdot
    
    # Build output tensor y (num_envs, 4, 1)
    y = torch.zeros(env.num_envs, 4, 1, device=env.device)
    y[:, 0, 0] = x[:, 5, 0]  # bXdot
    y[:, 1, 0] = x[:, 6, 0]  # bYdot
    y[:, 2, 0] = x[:, 7, 0]  # bZdot
    y[:, 3, 0] = x[:, 4, 0]  # bWz
    
    # Error calculations
    e = y - r
    E = e * env.step_dt  # Simplified - actual cumulative error would need tracking
    
    # Control signal calculation (batched)
    u = Hs - torch.bmm(Kx.expand(env.num_envs, -1, -1), x) - torch.bmm(Ky.expand(env.num_envs, -1, -1), E)
    
    # Rotor speed saturation (vectorized)
    u = torch.clamp(u, min=w_min, max=w_max)
    
    # Reorder rotors (NW, NE, SW, SE) -> (NE, NW, SE, SW)
    w_rotors = torch.zeros(env.num_envs, 4, device=env.device)
    w_rotors[:, 0] = u[:, 1, 0]  # NE
    w_rotors[:, 1] = u[:, 0, 0]  # NW
    w_rotors[:, 2] = u[:, 3, 0]  # SE
    w_rotors[:, 3] = u[:, 2, 0]  # SW
    
    # Calculate difference with actions
    scaled_actions = env.action_manager.action.abs() * action_scale
    diff = (w_rotors - scaled_actions).abs().sum(dim=1)
    
    # Apply special cases
    rewards[:] = 10 / (1 + torch.exp(torch.log(diff)))
    
    return rewards

def ang_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    """Penalize angular velocity deviation from a target value."""
    obs = env.obs_buf
    ang_vel_z = obs["policy"][:, 5]
    vel_command = env.command_manager.get_command("vel_command")
    rewards = torch.zeros(env.num_envs, device=env.device)

    diff = (ang_vel_z[:] - vel_command[:, 3]).abs()
    rewards[:] = 10 / (1 + torch.exp(torch.log(diff)))

    return rewards

def roll_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    """Penalize roll and pitch deviation from a target value."""
    roll = env.obs_buf["policy"][:, 6]
    target = torch.tensor(target, device=env.device)

    diff = (roll[:] - target).abs()

    return diff

def pitch_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    """Penalize roll and pitch deviation from a target value."""
    pitch = env.obs_buf["policy"][:, 7]
    target = torch.tensor(target, device=env.device)

    diff = (pitch[:] - target).abs()

    return diff