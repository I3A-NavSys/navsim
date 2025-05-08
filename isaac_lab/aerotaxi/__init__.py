# from .aerotaxi_env_reg import *
import gymnasium as gym
from . import env_hover, env_z_rotation, env_command

gym.register(
    id="Isaac-Hover-Aerotaxi-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{env_hover.__name__}.env_0.env:UAVEnvCfg",
        "rl_games_cfg_entry_point": f"{env_hover.__name__}.env0:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{env_hover.__name__}.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
        "skrl_cfg_entry_point": f"{env_hover.__name__}.env0:skrl_ppo_cfg.yaml",
        "sb3_cfg_entry_point": f"{env_hover.__name__}.env0:sb3_ppo_cfg.yaml",
    },
)

gym.register(
    id="Isaac-Z-Rotation-Aerotaxi-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{env_z_rotation.__name__}.env_0.env:UAVEnvCfg",
        "rsl_rl_cfg_entry_point": f"{env_z_rotation.__name__}.env_0.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Command-Aerotaxi-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{env_command.__name__}.env_0.env:UAVEnvCfg",
        "rsl_rl_cfg_entry_point": f"{env_command.__name__}.env_0.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Command-Aerotaxi-v1",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{env_command.__name__}.env_1.env:UAVEnvCfg",
        "rsl_rl_cfg_entry_point": f"{env_command.__name__}.env_1.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Command-Aerotaxi-v2",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{env_command.__name__}.env_2.env:UAVEnvCfg",
        "rsl_rl_cfg_entry_point": f"{env_command.__name__}.env_2.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
    },
)