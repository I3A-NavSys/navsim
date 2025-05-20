from __future__ import annotations

import torch
from typing import TYPE_CHECKING
from .flight_plan import FlightPlan

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

fp_change_lev_and_dir: FlightPlan = FlightPlan()
# Takeoff
fp_change_lev_and_dir.set_waypoint(time=10.0, pos=[0.0, 0.0, 1.75], vel=[0.0, 0.0, 0], heading=[1.0, 0.0])
# Straight line
fp_change_lev_and_dir.set_waypoint(time=30.0, pos=[100.0, 0.0, 60.0], vel=[10.0, 0.0, 0.0])
fp_change_lev_and_dir.set_waypoint(time=40.0, pos=[200.0, 0.0, 60.0], vel=[10.0, 0.0, 0.0])
fp_change_lev_and_dir.set_waypoint(time=50.0, pos=[300.0, 0.0, 60.0], vel=[10.0, 0.0, 0.0])
fp_change_lev_and_dir.set_waypoint(time=60.0, pos=[400.0, 0.0, 60.0], vel=[10.0, 0.0, 0.0])
fp_change_lev_and_dir.set_waypoint(time=70.0, pos=[500.0, 0.0, 60.0], vel=[10.0, 0.0, 0.0])
# Left curve
fp_change_lev_and_dir.set_waypoint(time=100.0, pos=[650.0, 150.0, 100.0], vel=[0.0, 10.0, 0.0])
# Straight line
fp_change_lev_and_dir.set_waypoint(time=110.0, pos=[650.0, 250.0, 100.0], vel=[0.0, 10.0, 0.0])
# Left curve
fp_change_lev_and_dir.set_waypoint(time=120.0, pos=[650.0, 350.0, 100.0], vel=[0.0, 10.0, 0.0])
# Straight line
fp_change_lev_and_dir.set_waypoint(time=150.0, pos=[500.0, 500.0, 60.0], vel=[-10.0, 0.0, 0.0])
fp_change_lev_and_dir.set_waypoint(time=160.0, pos=[400.0, 500.0, 60.0], vel=[-10.0, 0.0, 0.0])
fp_change_lev_and_dir.set_waypoint(time=170.0, pos=[300.0, 500.0, 60.0], vel=[-10.0, 0.0, 0.0])
fp_change_lev_and_dir.set_waypoint(time=180.0, pos=[200.0, 500.0, 60.0], vel=[-10.0, 0.0, 0.0])
fp_change_lev_and_dir.set_waypoint(time=190.0, pos=[100.0, 500.0, 60.0], vel=[-10.0, 0.0, 0.0])
# Change direction
fp_change_lev_and_dir.set_waypoint(time=230.0, pos=[100.0, 600.0, 60.0], vel=[10.0, 0.0, 0.0])
# Straight line
fp_change_lev_and_dir.set_waypoint(time=240.0, pos=[200.0, 600.0, 60.0], vel=[10.0, 0.0, 0.0])
fp_change_lev_and_dir.set_waypoint(time=250.0, pos=[300.0, 600.0, 60.0], vel=[10.0, 0.0, 0.0])
fp_change_lev_and_dir.set_waypoint(time=260.0, pos=[400.0, 600.0, 60.0], vel=[10.0, 0.0, 0.0])
fp_change_lev_and_dir.set_waypoint(time=270.0, pos=[500.0, 600.0, 60.0], vel=[10.0, 0.0, 0.0])
# Landing
fp_change_lev_and_dir.set_waypoint(time=290.0, pos=[600.0, 600.0, 20.0], vel=[0.0, 0.0, -3.0], heading=[1.0, 0.0])
fp_change_lev_and_dir.set_waypoint(time=300.0, pos=[600.0, 600.0, 3.0], vel=[0.0, 0.0, -0.2], heading=[1.0, 0.0])
fp_change_lev_and_dir.set_waypoint(time=310.0, pos=[600.0, 600.0, 0.0], vel=[0.0, 0.0, 0.0], heading=[1.0, 0.0])
fp_change_lev_and_dir.connect_waypoints()

def rew_pos_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    current_pos = obs["policy"][:, :3]
    fp_status = fp_change_lev_and_dir.status_at_time(env.sim.current_time % fp_change_lev_and_dir.finish_time())
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_10 = torch.tensor(10, device=env.device)
    torch_2 = torch.tensor(2, device=env.device)

    diff = current_pos - torch.tensor(fp_status.pos, device=env.device)
    diff_norm = torch.norm(diff, dim=1)
    rewards[:] = torch_10 / (torch_2 ** diff_norm)

    return rewards

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
    torch_2 = torch.tensor(2, device=env.device)

    diff = (lin_vel - vel_command).abs()
    rewards[:] = torch_3 / (torch_2 ** diff)

    return rewards

def rew_y_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 4]
    vel_command = obs["policy"][:, 13]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_3 = torch.tensor(3, device=env.device)
    torch_2 = torch.tensor(2, device=env.device)

    diff = (lin_vel - vel_command).abs()
    rewards[:] = torch_3 / (torch_2 ** diff)

    return rewards

def rew_z_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 5]
    vel_command = obs["policy"][:, 14]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_3 = torch.tensor(3, device=env.device)
    torch_2 = torch.tensor(2, device=env.device)

    diff = (lin_vel - vel_command).abs()
    rewards[:] = torch_3 / (torch_2 ** diff)

    return rewards

def rew_z_ang_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    ang_vel_z = obs["policy"][:, 8]
    vel_command = obs["policy"][:, 15]
    rewards = torch.zeros(env.num_envs, device=env.device)
    torch_10 = torch.tensor(10, device=env.device)
    torch_2 = torch.tensor(2, device=env.device)

    diff = (ang_vel_z - vel_command).abs()
    rewards[:] = torch_10 / (torch_2 ** diff)

    return rewards

def pen_pos_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    current_pos = obs["policy"][:, :3]
    fp_status = fp_change_lev_and_dir.status_at_time(env.sim.current_time % fp_change_lev_and_dir.finish_time())

    diff = current_pos - torch.tensor(fp_status.pos, device=env.device)
    diff_norm = torch.norm(diff, dim=1)

    return diff_norm

def pen_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 3:6]
    vel_command = obs["policy"][:, 12:15]

    diff = lin_vel[:] - vel_command[:]
    diff_norm = torch.norm(diff, dim=1)

    return diff_norm

def pen_x_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 3]
    vel_command = obs["policy"][:, 12]

    diff = (lin_vel - vel_command).abs()

    return diff

def pen_y_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 4]
    vel_command = obs["policy"][:, 13]

    diff = (lin_vel - vel_command).abs()

    return diff

def pen_z_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 5]
    vel_command = obs["policy"][:, 14]

    diff = (lin_vel - vel_command).abs()

    return diff

def pen_z_ang_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    ang_vel_z = obs["policy"][:, 8]
    vel_command = obs["policy"][:, 15]

    diff = (ang_vel_z[:] - vel_command[:]).abs()

    return diff

def pen_roll_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    roll = env.obs_buf["policy"][:, 9]
    target = torch.tensor(target, device=env.device)

    diff = (roll[:] - target).abs()
    return diff

def pen_pitch_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    pitch = env.obs_buf["policy"][:, 10]
    target = torch.tensor(target, device=env.device)

    diff = (pitch[:] - target).abs()

    return diff

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