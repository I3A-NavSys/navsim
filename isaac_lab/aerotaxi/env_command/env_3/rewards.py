from __future__ import annotations

import torch
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

def rew_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 3:6]
    vel_command = obs["policy"][:, 12:15]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_10 = torch.tensor(10, device=env.device)
    torch_2 = torch.tensor(2, device=env.device)

    diff = lin_vel[:] - vel_command[:]
    diff_norm = torch.norm(diff, dim=1)
    rewards[:] = torch_10 / (torch_2 ** diff_norm)

    return rewards

def rew_x_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 3]
    vel_command = obs["policy"][:, 12]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_3 = torch.tensor(3, device=env.device)
    torch_1 = torch.tensor(1, device=env.device)

    diff = (lin_vel - vel_command).abs()
    rewards[:] = torch_3 / (torch_1 + torch.exp(torch.log(diff)))

    return rewards

def rew_y_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 4]
    vel_command = obs["policy"][:, 13]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_3 = torch.tensor(3, device=env.device)
    torch_1 = torch.tensor(1, device=env.device)

    diff = (lin_vel - vel_command).abs()
    rewards[:] = torch_3 / (torch_1 + torch.exp(torch.log(diff)))

    return rewards

def rew_z_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 5]
    vel_command = obs["policy"][:, 14]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_3 = torch.tensor(3, device=env.device)
    torch_1 = torch.tensor(1, device=env.device)

    diff = (lin_vel - vel_command).abs()
    rewards[:] = torch_3 / (torch_1 + torch.exp(torch.log(diff)))

    return rewards

def rew_xy_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 3:5]
    vel_command = obs["policy"][:, 12:14]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_10 = torch.tensor(10, device=env.device)
    torch_2 = torch.tensor(2, device=env.device)

    diff = lin_vel[:] - vel_command[:]
    diff_norm = torch.norm(diff, dim=1)
    rewards[:] = torch_10 / (torch_2 ** diff_norm)

    return rewards

def rew_z_ang_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    ang_vel_z = obs["policy"][:, 8]
    vel_command = obs["policy"][:, 15]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_10 = torch.tensor(10, device=env.device)
    # torch_1 = torch.tensor(1, device=env.device)
    torch_2 = torch.tensor(2, device=env.device)

    diff = (ang_vel_z - vel_command).abs()
    # rewards[:] = torch_10 / (torch_1 + torch.exp(torch.log(diff)))
    rewards[:] = torch_10 / (torch_2 ** diff)

    return rewards

# -- When command components are 0 --
def pen_x_lin_vel(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    x_lin_vel = obs["policy"][:,3]
    x_vel_command = obs["policy"][:, 12]    # X-linear velocity
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    x_zero_mask = x_vel_command[:] == torch_0
    # diff = (x_lin_vel[:] - x_vel_command[:]).abs()
    rewards[x_zero_mask] = x_lin_vel[x_zero_mask].abs()

    return rewards

def pen_y_lin_vel(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    y_lin_vel = obs["policy"][:, 4]
    y_vel_command = obs["policy"][:, 13]    # Y-linear velocity
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    y_zero_mask = y_vel_command[:] == torch_0
    # diff = (y_lin_vel[:] - y_vel_command[:]).abs()
    rewards[y_zero_mask] = y_lin_vel[y_zero_mask].abs()

    return rewards

def pen_z_lin_vel(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    z_lin_vel = obs["policy"][:, 5]
    z_vel_command = obs["policy"][:, 14]    # Z-linear velocity
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    z_zero_mask = z_vel_command[:] == torch_0
    # diff = (z_lin_vel[:] - z_vel_command[:]).abs()
    rewards[z_zero_mask] = z_lin_vel[z_zero_mask].abs()

    return rewards

def pen_xy_lin_vel(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    xy_lin_vel = obs["policy"][:, 3:5]
    xy_vel_command = obs["policy"][:, 12:14]    # XY-linear velocity
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    xy_zero_mask = torch.sum(xy_vel_command, dim=1) == torch_0
    diff = torch.norm(xy_lin_vel[:] - xy_vel_command[:], dim=1)
    rewards[xy_zero_mask] = diff[xy_zero_mask]

    return rewards

def pen_xz_lin_vel(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    obs_mask = torch.tensor([0,0,0,1,0,1,0,0,0,0,0,0,1,0,1,0], device=env.device).bool()
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
    yz_lin_vel = obs["policy"][:, 4:6]
    yz_vel_command = obs["policy"][:, 13:15]    # YZ-linear velocity
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    yz_zero_mask = torch.sum(yz_vel_command, dim=1) == torch_0
    diff = torch.norm(yz_lin_vel[:] - yz_vel_command[:], dim=1)
    rewards[yz_zero_mask] = diff[yz_zero_mask]

    return rewards

def pen_xyz_lin_vel(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    xyz_lin_vel = obs["policy"][:, 3:6]
    xyz_vel_command = obs["policy"][:, 12:15]    # YZ-linear velocity
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    xyz_zero_mask = torch.sum(xyz_vel_command, dim=1) == torch_0
    diff = torch.norm(xyz_lin_vel[:] - xyz_vel_command[:], dim=1)
    rewards[xyz_zero_mask] = diff[xyz_zero_mask]

    return rewards

def pen_xy_roll_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    """Penalize roll deviation from a target value."""
    obs = env.obs_buf
    roll = obs["policy"][:, 9]
    vel_command = obs["policy"][:, 12:14]    # Angular velocity and Z-linear velocity excluded
    rewards = torch.zeros(env.num_envs, device=env.device)
    target = torch.tensor(target, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    xy_zero_mask = torch.sum(vel_command, dim=1) == torch_0
    diff = (roll[:] - target).abs()
    rewards[xy_zero_mask] = diff[xy_zero_mask]

    return rewards

def pen_xy_pitch_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    """Penalize pitch deviation from a target value."""
    obs = env.obs_buf
    pitch = obs["policy"][:, 10]
    vel_command = obs["policy"][:, 12:14]    # Angular velocity and Z-linear velocity excluded
    rewards = torch.zeros(env.num_envs, device=env.device)
    target = torch.tensor(target, device=env.device)
    torch_0 = torch.tensor(0.0, device=env.device)

    xy_zero_mask = torch.sum(vel_command, dim=1) == torch_0
    diff = (pitch[:] - target).abs()
    rewards[xy_zero_mask] = diff[xy_zero_mask]

    return rewards

def pen_roll_excess(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    """Penalize roll excess from target limit"""
    roll = env.obs_buf["policy"][:, 9]
    rewards = torch.zeros(env.num_envs, device=env.device)
    target = torch.tensor(target, device=env.device)
    
    excess_mask = roll[:] > target
    diff = (roll[:] - target).abs()
    rewards[excess_mask] = diff[excess_mask]
    
    return rewards

def pen_pitch_excess(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    """Penalize roll excess from target limit"""
    pitch = env.obs_buf["policy"][:, 10]
    rewards = torch.zeros(env.num_envs, device=env.device)
    target = torch.tensor(target, device=env.device)
    
    excess_mask = pitch[:] > target
    diff = (pitch[:] - target).abs()
    rewards[excess_mask] = diff[excess_mask]
    
    return rewards