from __future__ import annotations

# import os
# import sys


# project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# if project_root_path not in sys.path:
#     sys.path.append(project_root_path)

# import argparse

# from isaaclab.app import AppLauncher

# # add argparse arguments
# parser = argparse.ArgumentParser(description="Tutorial on creating a cartpole base environment.")
# parser.add_argument("--num_envs", type=int, default=16, help="Number of environments to spawn.")
# parser.add_argument("--task", type=str, default=None, help="Name of the task.")

# # append AppLauncher cli args
# AppLauncher.add_app_launcher_args(parser)
# # parse the arguments
# args_cli = parser.parse_args()

# # launch omniverse app
# app_launcher = AppLauncher(args_cli)
# simulation_app = app_launcher.app

"""Rest everything follows."""

import math
import torch
import numpy as np

# import isaaclab.envs.mdp as mdp
from . import mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, Articulation, ArticulationCfg
from isaaclab.envs import ManagerBasedRLEnv, ManagerBasedRLEnvCfg
from isaaclab.managers import ActionTermCfg, ActionTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.actuators import DCMotorCfg


class UAVactionTerm(ActionTerm):
    """Action term for the UAV."""

    # _asset: RigidObject
    _asset: Articulation

    def __init__(self, cfg: UAVactionTermCfg, env: ManagerBasedRLEnv):
        super().__init__(cfg, env)
        self._raw_actions = torch.zeros(env.num_envs, 4, device=self.device)
        self._processed_actions = torch.zeros(env.num_envs, 10, 3, device=self.device)


    @property
    def action_dim(self) -> int:
        return self._raw_actions.shape[1]

    @property
    def raw_actions(self) -> torch.Tensor:
        return self._raw_actions

    @property
    def processed_actions(self) -> torch.Tensor:
        return self._processed_actions

    def process_actions(self, actions: torch.Tensor):
        print(f"[DEBUG]: actions: {actions}")
        kFT_N = 4.6544
        kFT_S = 0.9309
        kFDx = 3.0625
        kFDy = 4.0000
        kFDz = 7.8400
        kMDR_N = 5.9683
        kMDR_S = 1.4921
        kMDx = 37.4010
        kMDy = 25.8580
        kMDz = 20.2514

        self._raw_actions[:] = actions

        lin_vels = self._asset.data.root_com_lin_vel_b
        ang_vels = self._asset.data.root_com_ang_vel_b

        for i, action in enumerate(self._raw_actions):
            lin_vel = lin_vels[i]
            ang_vel = ang_vels[i]
            
            # Apply thrust force
            FT_NW = [0, 0, kFT_N * action[0]**2]
            FT_NE = [0, 0, kFT_N * action[1]**2]
            FT_SW = [0, 0, kFT_S * action[2]**2]
            FT_SE = [0, 0, kFT_S * action[3]**2]        

            # Apply the air friction force to the drone
            FD = [-kFDx * lin_vel[0] * abs(lin_vel[0]),
                  -kFDy * lin_vel[1] * abs(lin_vel[1]),
                  -kFDz * lin_vel[2] * abs(lin_vel[2])]
        
            # Compute the drag moment
            MDR_NW = kMDR_N * action[0]**2
            MDR_NE = kMDR_N * action[1]**2
            MDR_SW = kMDR_S * action[2]**2
            MDR_SE = kMDR_S * action[3]**2
            MDR = [0, 0, MDR_NE - MDR_NW - MDR_SE + MDR_SW]

            # Compute the air friction moment
            MD = [-kMDx * ang_vel[0] * abs(ang_vel[0]),
                  -kMDy * ang_vel[1] * abs(ang_vel[1]),
                  -kMDz * ang_vel[2] * abs(ang_vel[2])]
            
            torque = [MDR[0] + MD[0], MDR[1] + MD[1], MDR[2] + MD[2]]
            zero_torque = [0, 0, 0]

            self._processed_actions[i][0][:] = torch.tensor(FD, device=self.device)
            self._processed_actions[i][1][:] = torch.tensor(FT_NW, device=self.device)
            self._processed_actions[i][2][:] = torch.tensor(FT_NE, device=self.device)
            self._processed_actions[i][3][:] = torch.tensor(FT_SW, device=self.device)
            self._processed_actions[i][4][:] = torch.tensor(FT_SE, device=self.device)
            self._processed_actions[i][5][:] = torch.tensor(torque, device=self.device)
            self._processed_actions[i][6][:] = torch.tensor(zero_torque, device=self.device)
            self._processed_actions[i][7][:] = torch.tensor(zero_torque, device=self.device)
            self._processed_actions[i][8][:] = torch.tensor(zero_torque, device=self.device)
            self._processed_actions[i][9][:] = torch.tensor(zero_torque, device=self.device)

            # print(f"[DEBUG]: Forces - {self._processed_actions[0][:5]}")

    def apply_actions(self):
        # print(f"[DEBUG]: Forces - {self._processed_actions[:, :5, :]}")
        # print(f"[DEBUG]: Torques - {self._processed_actions[:, 5:, :]}")

        positions = torch.tensor([[0,0,0], [0.5, 1.95, 0.5], [0.5, -1.95, 0.5], [-2.5, 1.55, 0.5], [-2.5, -1.55, 0.5]], 
                                 device=self.device)
        indices = torch.tensor(range(self._processed_actions.size(0)), device=self.device)

        self._asset.root_physx_view.apply_forces_and_torques_at_position(force_data=self._processed_actions[:, :5, :], 
                                                                         torque_data=self._processed_actions[:, 5:, :],
                                                                         position_data=positions,
                                                                         indices=indices,
                                                                         is_global=False)

        # This can be used to simulate wind forces it seems
        # mdp.apply_external_force_torque()


@configclass
class UAVactionTermCfg(ActionTermCfg):
    """Action term configuration for the UAV."""

    class_type: type = UAVactionTerm
    """Class type of the action term."""


@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    uav_pos = UAVactionTermCfg(asset_name="aerotaxi")


def my_obs_pos(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    # asset: RigidObject = env.scene[asset_cfg.name]
    asset: Articulation = env.scene[asset_cfg.name]
    pos = asset.data.root_pos_w
    return pos

def my_obs_lin_vel(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    # asset: RigidObject = env.scene[asset_cfg.name]
    asset: Articulation = env.scene[asset_cfg.name]
    lin_vel = asset.data.root_com_lin_vel_b
    return lin_vel

def my_obs_ang_vel(env: ManagerBasedRLEnv, asset_cfg: SceneEntityCfg):
    # asset: RigidObject = env.scene[asset_cfg.name]
    asset: Articulation = env.scene[asset_cfg.name]
    ang_vel = asset.data.root_com_ang_vel_b
    return ang_vel

@configclass
class ObervervationCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observation group for the policy."""
        pos = ObsTerm(func=my_obs_pos, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        lin_vel = ObsTerm(func=my_obs_lin_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})
        ang_vel = ObsTerm(func=my_obs_ang_vel, params={"asset_cfg": SceneEntityCfg(name="aerotaxi")})

        def __pos_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = False

    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Event specifications for the environment."""

    reset_pos = EventTerm(
        func=mdp.reset_root_state_uniform, 
        mode="reset",
        params={
            "pose_range": {
                "x": (-1, 1), 
                "y": (-1, 1), 
                # "roll": (0, 0),
                # "pitch": (0, 0),
                "roll": (-1.57, 1.57),
                "pitch": (-1.57, 1.57),
                "yaw": (-3.14, 3.14)
            },
            "velocity_range": {
                "x": (0, 0),
                "y": (0, 0),
                "z": (0, 0)
            },
            "asset_cfg": SceneEntityCfg(name="aerotaxi")
        }
    )

@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # (1) Constant running reward
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    # (2) Failure penalty
    terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)
    # (3) Primary task: hover
    pole_pos = RewTerm(
        func=mdp.lin_vel_diff,
        weight=-1.0,
        params={
            "asset_cfg": SceneEntityCfg("aerotaxi", joint_names=["NW_joint", "NE_joint", "SW_joint", "SE_joint"]), 
            "target": [0.0, 0.0, 0.0]
        },
    )

@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # (2) Linear velocity in z direction exceeds a negative threshold
    len_vel_z_out_bounds = DoneTerm(
        func=mdp.lin_vel_z_termination,
        params={"asset_cfg": SceneEntityCfg("aerotaxi", joint_names=["NW_joint", "NE_joint", "SW_joint", "SE_joint"]), },
    )

@configclass
class MySceneCfg(InteractiveSceneCfg):
    """Configuration for a UAV scene"""

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100, 100))
    )

    aerotaxi: ArticulationCfg = ArticulationCfg(
        prim_path="{ENV_REGEX_NS}/aerotaxi",
        spawn=sim_utils.UsdFileCfg(
            usd_path="C:/Users/Victor/Desktop/NAVSIM_GROUP/NAVSIM/isaac_lab/UAM_aerotaxi_lab.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                rigid_body_enabled=True,
                max_linear_velocity=1000.0,
                max_angular_velocity=1000.0,
                max_depenetration_velocity=100.0,
                enable_gyroscopic_forces=True,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=False,
                solver_position_iteration_count=4,
                solver_velocity_iteration_count=0,
                sleep_threshold=0.005,
                stabilization_threshold=0.001,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0, 0, 20),
            joint_pos={
                "NW_joint": 0.0,
                "NE_joint": 0.0,
                "SW_joint": 0.0,
                "SE_joint": 0.0,
            },
        ),
        actuators={
            "NW_rotor": DCMotorCfg(
                joint_names_expr=["NW_joint"],
                effort_limit=400.0,
                velocity_limit=100.0,
                stiffness=0.0,
                damping=10.0,
                saturation_effort=1000.0,
            ),
            "NE_rotor": DCMotorCfg(
                joint_names_expr=["NE_joint"],
                effort_limit=400.0,
                velocity_limit=100.0,
                stiffness=0.0,
                damping=10.0,
                saturation_effort=1000.0,
            ),
            "SW_rotor": DCMotorCfg(
                joint_names_expr=["SW_joint"],
                effort_limit=400.0,
                velocity_limit=100.0,
                stiffness=0.0,
                damping=10.0,
                saturation_effort=1000.0,
            ),
            "SE_rotor": DCMotorCfg(
                joint_names_expr=["SE_joint"],
                effort_limit=400.0,
                velocity_limit=100.0,
                stiffness=0.0,
                damping=10.0,
                saturation_effort=1000.0,
            ),
        }
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(
            color=(0.75, 0.75, 0.75),
            intensity=30000
        )
    )

@configclass
class UAVEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the UAV environment."""

    # Scene settings
    scene: MySceneCfg = MySceneCfg(num_envs=32, env_spacing=10, replicate_physics=False)
    
    # Basic settings
    observations: ObervervationCfg = ObervervationCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    def __post_init__(self):
        """Post initialization"""
        # viewer settings
        self.viewer.eye = [4.5, 0.0, 6.0]
        self.viewer.lookat = [0.0, 0.0, 2.0]
        # step settings
        self.decimation = 4  # env step every 4 sim steps: 200Hz / 4 = 50Hz
        self.episode_length_s = 10.0  # 10s
        # simulation settings
        self.sim.dt = 0.005  # sim step every 5ms: 200Hz
        self.sim.render_interval = self.decimation

# def main():
#     """Main function."""
    
#     # Setup base environment
#     env = ManagerBasedRLEnv(cfg=UAVEnvCfg())

#     # Setup target velocity command
#     target_rotor_vel = torch.zeros(env.num_envs, 4, device=env.device)
#     target_rotor_vel[:, 0] = 41.8879
#     target_rotor_vel[:, 1] = 41.8879
#     target_rotor_vel[:, 2] = 41.8879
#     target_rotor_vel[:, 3] = 41.8879

#     # Simulate physics
#     count = 0
#     env.reset()   # Extra info is a dictionary with more information

#     while simulation_app.is_running():
#         with torch.inference_mode():
#             # Reset
#             if count % 200 == 0:
#                 count = 0
#                 env.reset()
#                 print("-" * 80)
#                 print("[INFO]: Resetting the environment...")

#             # Step env
#             obs, rew, terminated, truncated, info = env.step(target_rotor_vel)

#             print(rew)

#             # print(f"[Step: {count:04d}]: Linear velocity[0]: {obs['policy'][0, 3:6]}")
#             # print(f"[Step: {count:04d}]: Angular velocity[0]: {obs['policy'][0, 6:9]}")

#             # Update counter
#             count += 1

#     # Close the environment
#     env.close()


# if __name__ == "__main__":
#     main()
#     simulation_app.close()