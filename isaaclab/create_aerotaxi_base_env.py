from __future__ import annotations

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on creating a cartpole base environment.")
parser.add_argument("--num_envs", type=int, default=16, help="Number of environments to spawn.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import math
import torch

import isaaclab.envs.mdp as mdp
import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObject, RigidObjectCfg
from isaaclab.envs import ManagerBasedEnv, ManagerBasedEnvCfg
from isaaclab.managers import ActionTermCfg, ActionTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass


class UAVactionTerm(ActionTerm):
    """Action term for the UAV."""

    _asset: RigidObject

    def __init__(self, cfg: UAVactionTermCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)
        self._raw_actions = torch.zeros(env.num_envs, 3, device=self.device)
        self._processed_actions = torch.zeros(env.num_envs, 3, device=self.device)
        self._vel_command = torch.zeros(env.num_envs, 6, device=self.device)
        # self._processed_actions = torch.zeros(env.num_envs, 1, 6, device=self.device)   # 1 is the number of body_ids
        # gains of controller
        self.p_gain = cfg.p_gain
        self.d_gain = cfg.d_gain

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
        # kFT_N = 4.6544
        # kFT_S = 0.9309
        # kFDx = 3.0625
        # kFDy = 4.0000
        # kFDz = 7.8400
        # kMDR_N = 5.9683
        # kMDR_S = 1.4921
        # kMDx = 37.4010
        # kMDy = 25.8580
        # kMDz = 20.2514

        # self._raw_actions[:] = actions

        # lin_vels = self._asset.data.root_com_lin_vel_b
        # ang_vels = self._asset.data.root_com_ang_vel_b

        # for i, action in enumerate(self._raw_actions):
        #     lin_vel = lin_vels[i]
        #     ang_vel = ang_vels[i]
            
        #     # Apply thrust force
        #     FT_NE = [0, 0, kFT_N * action[0]**2]
        #     FT_NW = [0, 0, kFT_N * action[1]**2]
        #     FT_SE = [0, 0, kFT_S * action[2]**2]
        #     FT_SW = [0, 0, kFT_S * action[3]**2]        

        #     # Apply the air friction force to the drone
        #     FD = [-kFDx * lin_vel[0] * abs(lin_vel[0]),
        #           -kFDy * lin_vel[1] * abs(lin_vel[1]),
        #           -kFDz * lin_vel[2] * abs(lin_vel[2])]
        
        #     # Compute the drag moment
        #     MDR_NE = kMDR_N * action[0]**2
        #     MDR_NW = kMDR_N * action[1]**2
        #     MDR_SE = kMDR_S * action[2]**2
        #     MDR_SW = kMDR_S * action[3]**2
        #     MDR = [0, 0, MDR_NE - MDR_NW - MDR_SE + MDR_SW]

        #     # Compute the air friction moment
        #     MD = [-kMDx * ang_vel[0] * abs(ang_vel[0]),
        #           -kMDy * ang_vel[1] * abs(ang_vel[1]),
        #           -kMDz * ang_vel[2] * abs(ang_vel[2])]
            
        #     torque = [MDR[0] + MD[0], MDR[1] + MD[1], MDR[2] + MD[2]]

        #     # self._processed_actions[i][0] = torch.tensor(FT_NE, device=self.device)
        #     # self._processed_actions[i][1] = torch.tensor(FT_NW, device=self.device)
        #     # self._processed_actions[i][2] = torch.tensor(FT_SE, device=self.device)
        #     # self._processed_actions[i][3] = torch.tensor(FT_SW, device=self.device)
        #     self._processed_actions[i][0][:3] = torch.tensor(FD, device=self.device)
        #     self._processed_actions[i][0][3:6] = torch.tensor(torque, device=self.device)
        self._raw_actions[:] = actions
        self._processed_actions[:] = self._raw_actions[:]

    def apply_actions(self):
        pos_error = self._processed_actions - self._asset.data.root_pos_w
        vel_error = -self._asset.data.root_lin_vel_w
        # self._vel_command[:, :3] = pos_error
        self._vel_command[:, :3] = self.p_gain * pos_error + self.d_gain * vel_error
        self._asset.write_root_velocity_to_sim(self._vel_command)

        
        # self._asset.set_external_force_and_torque(self._processed_actions[:, :, :3], self._processed_actions[:, :, 3:6])
        # self._asset.write_data_to_sim()

        # This can be used to simulate wind forces it seems
        # mdp.apply_external_force_torque()


@configclass
class UAVactionTermCfg(ActionTermCfg):
    """Action term configuration for the UAV."""

    class_type: type = UAVactionTerm
    """Class type of the action term."""

    p_gain: float = 5.0
    """Proportional gain of the PD controller."""
    d_gain: float = 0.5
    """Derivative gain of the PD controller."""


@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    uav_pos = UAVactionTermCfg(asset_name="aerotaxi")


def my_obs_pos(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg):
    asset: RigidObject = env.scene[asset_cfg.name]
    pos = asset.data.root_pos_w
    return pos

def my_obs_lin_vel(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg):
    asset: RigidObject = env.scene[asset_cfg.name]
    lin_vel = asset.data.root_com_lin_vel_b
    return lin_vel

def my_obs_ang_vel(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg):
    asset: RigidObject = env.scene[asset_cfg.name]
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
                "yaw": (-3.14, 3.14)
            },
            "velocity_range": {
                "x": (-2, 2),
                "y": (-2, 2),
                "z": (-2, 2)
            },
            "asset_cfg": SceneEntityCfg(name="aerotaxi")
        }
    )

@configclass
class MySceneCfg(InteractiveSceneCfg):
    """Configuration for a UAV scene"""

    ground = AssetBaseCfg(
        prim_path="/World/ground",
        spawn=sim_utils.GroundPlaneCfg(size=(100, 100))
    )

    aerotaxi: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/aerotaxi",
        spawn=sim_utils.UsdFileCfg(
            usd_path="C:/Users/Victor/Desktop/NAVSIM_GROUP/NAVSIM/tmp/fleet/UAM_aerotaxi_rpv/UAM_aerotaxi.usd",
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
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
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0, 0, 10))
    )

    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(
            color=(0.75, 0.75, 0.75),
            intensity=30000
        )
    )

@configclass
class UAVEnvCfg(ManagerBasedEnvCfg):
    """Configuration for the UAV environment."""

    # Scene settings
    scene: MySceneCfg = MySceneCfg(num_envs=args_cli.num_envs, env_spacing=10, replicate_physics=False)
    
    # Basic settings
    observations: ObervervationCfg = ObervervationCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()

    def __post_init__(self):
        """Post initialization"""
        # viewer settings
        self.viewer.eye = [4.5, 0.0, 6.0]
        self.viewer.lookat = [0.0, 0.0, 2.0]
        # step settings
        self.decimation = 4  # env step every 4 sim steps: 200Hz / 4 = 50Hz
        # simulation settings
        self.sim.dt = 0.005  # sim step every 5ms: 200Hz

def main():
    """Main function."""
    
    # Setup base environment
    env = ManagerBasedEnv(cfg=UAVEnvCfg())

    # Setup target velocity command
    target_pos = torch.zeros(env.num_envs, 3, device=env.device)
    target_pos[:, 0] = 0
    target_pos[:, 0] = 0
    target_pos[:, 2] = 50

    # Simulate physics
    count = 0
    obs, extra_info = env.reset()   # Extra info is a dictionary with more information

    while simulation_app.is_running():
        with torch.inference_mode():
            # Reset
            if count % 500 == 0:
                count = 0
                obs, extra_info = env.reset()
                print("-" * 80)
                print("[INFO]: Resetting the environment...")

            # Step env
            obs, extra_info = env.step(target_pos)

            error = torch.norm(obs["policy"][:, :3] - target_pos[:, :3]).mean().item()
            print(f"[Step: {count:04d}]: Mean pos error: {error:.4f}")

            # # Print positions
            # print(f"[Step: {count:04d}]: Position: {obs['policy'][:, :3]}")

            # # Print mean squared linear velocity error between target and current velocity
            # error = torch.norm(obs["policy"][:, 3:6] - target_vel[:, :3]).mean().item()
            # print(f"[Step: {count:04d}]: Mean linar velocity error: {error:.4f}")

            # # Print mean squared angular velocity error between target and current velocity
            # error = torch.norm(obs["policy"][:, 8] - target_vel[:, 3]).mean().item()
            # print(f"[Step: {count:04d}]: Mean angular velocity error: {error:.4f}")

            # Update counter
            count += 1

    # Close the environment
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()