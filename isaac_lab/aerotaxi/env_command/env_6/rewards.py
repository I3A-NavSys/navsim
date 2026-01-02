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
    fp_status = torch.tensor(fp_change_lev_and_dir.status_at_time(env.sim.current_time % fp_change_lev_and_dir.finish_time()), device=env.device)
    error = torch.norm(current_pos - fp_status, dim=1)
    return torch.clamp(error, max=5)  # como mucho 5 metros de diferencia


def rew_pos_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
    obs = env.obs_buf
    current_pos = obs["policy"][:, :3]
    fp_status = torch.tensor(fp_change_lev_and_dir.status_at_time(env.sim.current_time % fp_change_lev_and_dir.finish_time()), device=env.device)
    distance = torch.norm(current_pos - fp_status, dim=1)
    return 1 - torch.tanh(distance/std) 


def rew_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 3:6]  
    vel_command = obs["policy"][:, 12:15]
    error = torch.norm(lin_vel - vel_command, dim=1)
    return torch.clamp(error, max=20.0)  # evito recompensa infinita con máximo 20 m/s


def rew_lin_vel_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
    obs = env.obs_buf
    lin_vel = obs["policy"][:, 3:6]  
    vel_command = obs["policy"][:, 12:15]
    distance = torch.norm(lin_vel - vel_command, dim=1)
    return 1 - torch.tanh(distance/std)


def rew_ang_vel_z_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
    obs = env.obs_buf
    ang_vel = obs["policy"][:, 8]  
    ang_vel_command = obs["policy"][:, 15]
    error = torch.abs(ang_vel - ang_vel_command)
    return torch.clamp(error, max=10.0)  # maximo 10 rad/s


def rew_ang_vel_z_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
    obs = env.obs_buf
    ang_vel = obs["policy"][:, 8]  
    ang_vel_command = obs["policy"][:, 15]
    distance = torch.abs(ang_vel - ang_vel_command)
    return 1 - torch.tanh(distance/std)


def rew_roll_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    obs = env.obs_buf
    roll = obs["policy"][:, 9].squeeze()  
    roll_command = torch.tensor(target, device=env.device)
    error = torch.abs(roll - roll_command)
    return torch.clamp(error, max=torch.pi)


def rew_roll_diff_fine_grained(env: ManagerBasedRLEnv, target: float, std: float) -> torch.Tensor:
    obs = env.obs_buf
    roll = obs["policy"][:, 9].squeeze()  
    roll_command = torch.tensor(target, device=env.device)
    distance = torch.abs(roll - roll_command)
    return 1 - torch.tanh(distance/std)


def rew_pitch_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
    obs = env.obs_buf
    pitch = obs["policy"][:, 10].squeeze()  
    pitch_command = torch.tensor(target, device=env.device)
    error = torch.abs(pitch - pitch_command)
    return torch.clamp(error, max=torch.pi)


def rew_pitch_diff_fine_grained(env: ManagerBasedRLEnv, target: float, std: float) -> torch.Tensor:
    obs = env.obs_buf
    pitch = obs["policy"][:, 10].squeeze() 
    pitch_command = torch.tensor(target, device=env.device)
    distance = torch.abs(pitch - pitch_command)
    return 1 - torch.tanh(distance/std)
