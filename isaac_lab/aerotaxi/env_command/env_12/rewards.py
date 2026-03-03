from __future__ import annotations

import torch
from typing import TYPE_CHECKING
from .flight_plan import FlightPlan
import isaaclab.utils.math as math_utils

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

def rew_attitude_stability2(env):
    asset = env.scene["aerotaxi"]
    roll, pitch, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
    angle_term = torch.exp(-3.0 * (roll**2 + pitch**2))
    return angle_term

# def rew_ang_vel_stability3(env):
#     asset = env.scene["aerotaxi"]
#     ang_vel = asset.data.root_com_ang_vel_b
#     ang_vel_term = torch.exp(-0.5*(torch.norm(ang_vel, dim=1)**2))
#     return ang_vel_term 
def rew_ang_vel_stability4(env):
    asset = env.scene["aerotaxi"]
    ang_vel = asset.data.root_com_ang_vel_b

    # Separar yaw
    ang_vel_xy = ang_vel[:, :2]
    yaw_rate = ang_vel[:, 2]

    term_xy = torch.exp(-0.5 * torch.norm(ang_vel_xy, dim=1)**2)
    term_yaw = torch.exp(-1.5 * yaw_rate**2)

    return 0.7 * term_xy + 0.3 * term_yaw

# def rew_altitude_hold3(env):
#     asset = env.scene["aerotaxi"]
#     z = asset.data.root_com_pos_w[:, 2]
#     z_target = env.command_manager.get_term("vel_command").target_pos[:, 2]

#     return -5.0*((z - z_target)**2)

def rew_altitude_hold2(env):
    asset = env.scene["aerotaxi"]
    z = asset.data.root_com_pos_w[:, 2]
    z_target = env.command_manager.get_term("vel_command").target_pos[:, 2]

    return torch.exp(-2*((z - z_target)**2))

def rew_vel2(env:ManagerBasedRLEnv):
    asset = env.scene["aerotaxi"]
    vel_lin_b = asset.data.root_com_lin_vel_w[:, :3]
    vel = torch.norm(vel_lin_b, dim=1)
    return torch.exp(-1.0 * vel**2)

def rew_vertical_velocity(env):
    asset = env.scene["aerotaxi"]
    vz = asset.data.root_com_lin_vel_w[:, 2]
    return -3.0 * torch.clamp(-vz, min=0.0)**2

def rew_pos2(env: ManagerBasedRLEnv):
    term = env.command_manager.get_term("vel_command")
    target_pos = term.target_pos[:, :2] 
    asset = env.scene["aerotaxi"]
    current_pos_w = asset.data.root_com_pos_w
    current_pos_local = current_pos_w[:, :2] - env.scene.env_origins[:, :2]
    dist = torch.norm(current_pos_local - target_pos, dim=1)
    dist_sq = dist**2
    pos_term = torch.exp(-0.5 * dist_sq)
    return pos_term 

def rew_action_rate(env: ManagerBasedRLEnv) -> torch.Tensor:
    # Penaliza la diferencia entre la acción actual y la anterior
    diff = -0.01 * torch.norm(env.action_manager.action - env.action_manager.prev_action, dim=1)**2
    return diff


# def rew_action_rate2(env: ManagerBasedRLEnv) -> torch.Tensor:
#     # Penaliza la diferencia entre la acción actual y la anterior
#     action_rate = torch.norm(env.action_manager.action, dim=1)
#     return action_rate**2
# def rew_action_rate(env: ManagerBasedRLEnv) -> torch.Tensor:
#     # Penaliza la diferencia entre la acción actual y la anterior
#     diff = torch.norm(env.action_manager.action - env.action_manager.prev_action, dim=1)
#     return torch.clamp(diff, max=5.0)

# def rew_hover_stability(env):
#     term = env.command_manager.get_term("vel_command")
#     target_pos = term.target_pos 
#     asset = env.scene["aerotaxi"]

#     pos = asset.data.root_com_pos_w[:, :3] - env.scene.env_origins[:, :3]
#     vel = asset.data.root_com_lin_vel_w[:, :3]

#     dist = torch.norm(pos - target_pos, dim=1)
#     vel_norm = torch.norm(vel, dim=1)

#     pos_term = torch.exp(-(dist / 4.0)**2)
#     vel_term = torch.exp(-(vel_norm / 4.0)**2)

#     return pos_term * vel_term
    
# def rew_altitude_hold(env):
#     asset = env.scene["aerotaxi"]
#     z = asset.data.root_com_pos_w[:, 2]
#     z_target = env.command_manager.get_term("vel_command").target_pos[:, 2]

#     return torch.exp(-((z - z_target)**2) / (0.5**2))

# def rew_attitude_stability(env):
#     asset = env.scene["aerotaxi"]
#     roll, pitch, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
#     ang_vel = asset.data.root_com_ang_vel_b[:, :2]

#     angle_term = torch.exp(-(roll**2 + pitch**2) / (0.3**2))
#     ang_vel_term = torch.exp(-(torch.norm(ang_vel, dim=1)**2) / (0.5**2))

#     return angle_term * ang_vel_term

# def rew_vel(env:ManagerBasedRLEnv):
#     asset = env.scene["aerotaxi"]
#     vel_lin_b = asset.data.root_com_lin_vel_w[:, :3]
#     vel = torch.norm(vel_lin_b, dim=1)
#     return vel**2

# def rew_pos_fine(env: ManagerBasedRLEnv):
#     term = env.command_manager.get_term("vel_command")
#     target_pos = term.target_pos 
#     asset = env.scene["aerotaxi"]
#     current_pos_w = asset.data.root_com_pos_w
#     current_pos_local = current_pos_w[:, :3] - env.scene.env_origins[:, :3]
#     dist = torch.norm(current_pos_local - target_pos, dim=1)
#     vel_lin_b = asset.data.root_com_lin_vel_w[:, :3]
#     vel = torch.norm(vel_lin_b, dim=1)
#     pos_term = torch.exp(- (dist / 0.05)**2)
#     vel_term = torch.exp(- (vel / 0.05)**2)

#     return pos_term * vel_term


# def rew_pos_diff_cuad(env: ManagerBasedRLEnv):
#     term = env.command_manager.get_term("vel_command")
#     target_pos = term.target_pos 
#     current_pos_w = env.scene["aerotaxi"].data.root_com_pos_w
#     current_pos_local = current_pos_w[:, :3] - env.scene.env_origins[:, :3]

#     # distancia euclídea
#     dist = torch.norm(current_pos_local - target_pos, dim=1)

#     return dist**2

# def rew_ang_vel(env: ManagerBasedRLEnv):
#     asset = env.scene["aerotaxi"]
#     ang_vel_z = asset.data.root_com_ang_vel_b[:, :3]
#     error = torch.norm(ang_vel_z, dim=1)
#     return error**2

# def rew_tilt_penalty_pg(env: ManagerBasedRLEnv):
#     asset = env.scene["aerotaxi"]
#     tilt_error = torch.norm(asset.data.projected_gravity_b[:, :2], dim=1)
#     return tilt_error**2


# def rew_pos_diff(env: ManagerBasedRLEnv):
#     term = env.command_manager.get_term("vel_command")
#     target_pos = term.target_pos 
#     current_pos_w = env.scene["aerotaxi"].data.root_com_pos_w
#     current_pos_local = current_pos_w[:, :3] - env.scene.env_origins[:, :3]

#     # distancia euclídea
#     dist = torch.norm(current_pos_local - target_pos, dim=1)

#     return torch.exp(-dist / 2)

# def rew_vel_z(env: ManagerBasedRLEnv):
#     asset = env.scene["aerotaxi"]
#     vel_lin_b = asset.data.root_com_lin_vel_w[:, 2]
#     return torch.relu(-vel_lin_b, dim=1)

# def rew_pos_diff_exp(env: ManagerBasedRLEnv):
#     term = env.command_manager.get_term("vel_command")
#     target_pos = term.target_pos 
#     current_pos_w = env.scene["aerotaxi"].data.root_com_pos_w
#     current_pos_local = current_pos_w[:, :3] - env.scene.env_origins[:, :3]

#     # distancia euclídea
#     dist = torch.norm(current_pos_local - target_pos, dim=1)

#     return torch.exp(dist)

# def rew_pos_diff_xy(env: ManagerBasedRLEnv):
#     term = env.command_manager.get_term("vel_command")
#     target_pos = term.target_pos[:, :2]
#     current_pos_w = env.scene["aerotaxi"].data.root_com_pos_w
#     current_pos_local = current_pos_w[:, :2] - env.scene.env_origins[:, :2]

#     # distancia euclídea
#     dist = torch.norm(current_pos_local - target_pos, dim=1)

#     return torch.exp(-dist / 2)

# def rew_pos_diff_z(env: ManagerBasedRLEnv):
#     term = env.command_manager.get_term("vel_command")
#     target_pos = term.target_pos[:, 2] 
#     current_pos_w = env.scene["aerotaxi"].data.root_com_pos_w
#     current_pos_local = current_pos_w[:, 2] - env.scene.env_origins[:, 2]
#     dist = torch.abs(current_pos_local - target_pos)

#     return torch.exp(-dist / 2)

# def rew_emergency_climb(env: ManagerBasedRLEnv):
#     asset = env.scene["aerotaxi"]
#     command_term = env.command_manager.get_term("vel_command")
    
#     # 1. Altura
#     z_actual = asset.data.root_com_pos_w[:, 2] - env.scene.env_origins[:, 2]
#     z_target = command_term.target_pos[:, 2]
    
#     # 2. Velocidad vertical
#     vel_w_z = asset.data.root_com_lin_vel_w[:, 2]
    
#     # 3. Lógica: Si estoy bajo, premiar velocidad positiva Y penalizar la negativa
#     is_below = z_actual < z_target
    
#     # En lugar de clamp(0), permitimos valores negativos.
#     # Si vel_z = -2.0 (cayendo), dará -2.0.
#     # Si vel_z = 1.0 (subiendo), dará +1.0.
#     return is_below.float() * vel_w_z

# def rew_pos_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
#     term = env.command_manager.get_term("vel_command")
#     target_pos = term.target_pos 

#     current_pos_w = env.scene["aerotaxi"].data.root_com_pos_w
#     current_pos_local = current_pos_w[:, :3] - env.scene.env_origins[:, :3]
    
#     error = torch.norm(current_pos_local - target_pos, dim=1)
#     return torch.clamp(error, max=100.0)

# def rew_pos_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
#     term = env.command_manager.get_term("vel_command")
#     target_pos = term.target_pos
    
#     current_pos_w = env.scene["aerotaxi"].data.root_com_pos_w
#     current_pos_local = current_pos_w[:, :3] - env.scene.env_origins[:, :3]
    
#     distance = torch.norm(current_pos_local - target_pos, dim=1)
#     return 1.0 - torch.tanh(distance / std)

# def rew_pos_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
#     asset = env.scene["aerotaxi"]
#     command_term = env.command_manager.get_term("vel_command")

#     # Posición actual en world
#     pos_w = asset.data.root_com_pos_w[:, :3]
#     pos_local = pos_w - env.scene.env_origins[:, :3]

#     distance = torch.norm(command_term.target_pos - pos_local, dim=1)
#     return 1.0 - torch.tanh(distance / std)



# def rew_lin_vel_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
#     asset = env.scene["aerotaxi"]
#     lin_vel_b = asset.data.root_com_lin_vel_b
#     vel_command = env.command_manager.get_command("vel_command")[:, :3]
    
#     error = torch.norm(lin_vel_b - vel_command, dim=1)
#     return torch.clamp(error, max=10.0)

# def rew_lin_vel_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
#     asset = env.scene["aerotaxi"]
#     command_term = env.command_manager.get_term("vel_command")

#     lin_vel_b = asset.data.root_com_lin_vel_b

#     # Target velocity en world
#     target_vel_w = command_term.target_vel

#     # Convertimos target a body frame
#     quat_inv = math_utils.quat_inv(asset.data.root_com_quat_w)
#     target_vel_b = math_utils.quat_apply(quat_inv, target_vel_w)

    # pruebo a descomponer el error de velocidad en xy y en z, para dar más peso a la z
    # vel_error_xy = torch.norm(lin_vel_b[:, :2] - target_vel_b[:, :2], dim=1)
    # vel_error_z = torch.abs(lin_vel_b[:, 2] - target_vel_b[:, 2])
    # combined_error = vel_error_xy + (2 * vel_error_z)
    # return 1.0 - torch.tanh(combined_error / std)



# def rew_ang_vel_z_diff(env: ManagerBasedRLEnv) -> torch.Tensor:
#     asset = env.scene["aerotaxi"]
#     ang_vel_z = asset.data.root_com_ang_vel_b[:, 2]
#     yaw_rate_command = env.command_manager.get_command("vel_command")[:, 3]
#     error = torch.abs(ang_vel_z - yaw_rate_command)
#     return torch.clamp(error, max=5.0)

# def rew_ang_vel_z_diff_fine_grained(env: ManagerBasedRLEnv, std: float) -> torch.Tensor:
#     asset = env.scene["aerotaxi"]
#     command_term = env.command_manager.get_term("vel_command")

#     ang_vel_z = asset.data.root_com_ang_vel_b[:, 2]
#     target_yaw_rate = command_term.target_yaw

#     error = torch.abs(ang_vel_z - target_yaw_rate)
#     return 1.0 - torch.tanh(error / std)


# def rew_tilt_penalty(env: ManagerBasedRLEnv):
#     asset = env.scene["aerotaxi"]
#     roll, pitch, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
#     tilt = roll**2 + pitch**2
#     return tilt

# def rew_roll_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
#     asset = env.scene["aerotaxi"]
#     roll, _, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
#     error = torch.abs(roll - target)
#     return error


# def rew_roll_diff_fine_grained(env: ManagerBasedRLEnv, target: float, std: float) -> torch.Tensor:
#     asset = env.scene["aerotaxi"]
#     roll, _, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
#     distance = torch.abs(roll - target)
#     return 1 - torch.tanh(distance/std)


# def rew_pitch_diff(env: ManagerBasedRLEnv, target: float) -> torch.Tensor:
#     asset = env.scene["aerotaxi"]
#     _, pitch, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
#     error = torch.abs(pitch - target)
#     return error


# def rew_pitch_diff_fine_grained(env: ManagerBasedRLEnv, target: float, std: float) -> torch.Tensor:
#     asset = env.scene["aerotaxi"]
#     _, pitch, _ = math_utils.euler_xyz_from_quat(asset.data.root_com_quat_w)
#     distance = torch.abs(pitch - target)
#     return 1 - torch.tanh(distance/std)

# def rew_hovering(env: ManagerBasedRLEnv, min_altitude: float, max_altitude: float):
#     asset = env.scene["aerotaxi"]
#     z = asset.data.root_com_pos_w[:, 2] - env.scene.env_origins[:, 2]

#     inside = (z >= min_altitude) & (z <= max_altitude)
#     return inside.float()


# def rew_ang_vel_xy_penalty(env: ManagerBasedRLEnv) -> torch.Tensor:
#     asset = env.scene["aerotaxi"]
#     ang_vel_xy = asset.data.root_com_ang_vel_b[:, :2]
#     penalty = torch.sum(torch.square(ang_vel_xy), dim=1)
#     return torch.clamp(penalty, max=50.0)

# def rew_velocity_rescue_bidirectional(env: ManagerBasedRLEnv) -> torch.Tensor:
#     """Premia que la velocidad vertical tenga el mismo signo que el error de posición."""
#     asset = env.scene["aerotaxi"]
#     command_term = env.command_manager.get_term("vel_command")
#     z_actual = asset.data.root_com_pos_w[:, 2] - env.scene.env_origins[:, 2]
#     z_target = command_term.target_pos[:, 2]
#     z_error = z_target - z_actual
#     vel_z = asset.data.root_com_lin_vel_b[:, 2]
#     direction_reward = torch.sign(z_error) * vel_z
#     return torch.clamp(direction_reward, min=0.0)