from __future__ import annotations

import torch
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv


def lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, :3]
    vel_command = obs["policy"][:, 8:11]    # Angular velocity is not needed
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_10 = torch.tensor(10, device=env.device)
    torch_2 = torch.tensor(2, device=env.device)

    diff = lin_vel[:] - vel_command[:]
    diff_norm = torch.norm(diff, dim=1)
    rewards[:] = torch_10 / torch_2 ** diff_norm

    return rewards

def ang_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    ang_vel_z = obs["policy"][:, 5]
    vel_command = obs["policy"][:, 11]  # Linear velocity is not needed
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_10 = torch.tensor(10, device=env.device)
    torch_2 = torch.tensor(2, device=env.device)

    diff = (ang_vel_z[:] - vel_command[:]).abs()
    rewards[:] = torch_10 / torch_2 ** diff

    return rewards

def jerky_mov(env: ManagerBasedRLEnv) -> torch.Tensor:
    current_actions = env.action_manager.action.abs()
    prev_actions = env.action_manager.prev_action.abs()

    diff = current_actions - prev_actions
    diff_norm = torch.norm(diff, dim=1)

    return diff_norm

def pen_x_lin_vel(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    x_lin_vel = obs["policy"][:, 0]
    x_vel_command = obs["policy"][:, 8]    # X-linear velocity
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    x_zero_mask = x_vel_command[:] == torch_0
    # diff = (x_lin_vel[:] - x_vel_command[:]).abs()
    rewards[x_zero_mask] = x_lin_vel[:, x_zero_mask].abs()

    return rewards

def pen_y_lin_vel(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    y_lin_vel = obs["policy"][:, 1]
    y_vel_command = obs["policy"][:, 9]    # Y-linear velocity
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    y_zero_mask = y_vel_command[:] == torch_0
    # diff = (y_lin_vel[:] - y_vel_command[:]).abs()
    rewards[y_zero_mask] = y_lin_vel[y_zero_mask].abs()

    return rewards

def pen_z_lin_vel(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    z_lin_vel = obs["policy"][:, 2]
    z_vel_command = obs["policy"][:, 10]    # Z-linear velocity
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    z_zero_mask = z_vel_command[:] == torch_0
    # diff = (z_lin_vel[:] - z_vel_command[:]).abs()
    rewards[z_zero_mask] = z_lin_vel[z_zero_mask].abs()

    return rewards

def pen_xy_lin_vel(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    xy_lin_vel = obs["policy"][:, :2]
    xy_vel_command = obs["policy"][:, 8:10]    # XY-linear velocity
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    xy_zero_mask = torch.sum(xy_vel_command, dim=1) == torch_0
    diff = torch.norm(xy_lin_vel[:] - xy_vel_command[:], dim=1)
    rewards[xy_zero_mask] = diff[xy_zero_mask]

    return rewards

def pen_xz_lin_vel(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    obs_mask = torch.tensor([1,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0], device=env.device).bool()
    xz_lin_vel = obs["policy"][:, obs_mask]
    xz_prim = xz_lin_vel[:, :2]
    xz_command = xz_lin_vel[:, 2:]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    xz_zero_mask = torch.sum(xz_command, dim=1) == torch_0
    diff = torch.norm(xz_prim[:] - xz_command[:], dim=1)
    rewards[xz_zero_mask] = diff[xz_zero_mask]

    return rewards

def pen_yz_lin_vel(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    yz_lin_vel = obs["policy"][:, 1:3]
    yz_vel_command = obs["policy"][:, 9:11]    # YZ-linear velocity
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    yz_zero_mask = torch.sum(yz_vel_command, dim=1) == torch_0
    diff = torch.norm(yz_lin_vel[:] - yz_vel_command[:], dim=1)
    rewards[yz_zero_mask] = diff[yz_zero_mask]

    return rewards

def pen_xyz_lin_vel(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    xyz_lin_vel = obs["policy"][:, :3]
    xyz_vel_command = obs["policy"][:, 8:11]    # YZ-linear velocity
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    xyz_zero_mask = torch.sum(xyz_vel_command, dim=1) == torch_0
    diff = torch.norm(xyz_lin_vel[:] - xyz_vel_command[:], dim=1)
    rewards[xyz_zero_mask] = diff[xyz_zero_mask]

    return rewards

def roll_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    """Penalize roll and pitch deviation from a target value."""
    obs = env.obs_buf
    roll = obs["policy"][:, 6]
    vel_command = obs["policy"][:, 8:10]    # Angular velocity and Z-linear velocity excluded
    rewards = torch.zeros(env.num_envs, device=env.device)
    target = torch.tensor(target, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    xy_zero_mask = torch.sum(vel_command, dim=1) == torch_0
    diff = (roll[:] - target).abs()
    rewards[xy_zero_mask] = diff[xy_zero_mask]

    return rewards

def pitch_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    """Penalize roll and pitch deviation from a target value."""
    obs = env.obs_buf
    pitch = obs["policy"][:, 7]
    vel_command = obs["policy"][:, 8:10]    # Angular velocity and Z-linear velocity excluded
    rewards = torch.zeros(env.num_envs, device=env.device)
    target = torch.tensor(target, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    xy_zero_mask = torch.sum(vel_command, dim=1) == torch_0
    diff = (pitch[:] - target).abs()
    rewards[xy_zero_mask] = diff[xy_zero_mask]

    return rewards