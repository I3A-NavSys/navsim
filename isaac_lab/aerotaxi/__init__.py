# from .aerotaxi_env_reg import *
import gymnasium as gym
from . import config

gym.register(
    id="Isaac-Hover-Aerotaxi-v1",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.isaac_hover_aerotaxi_v1:UAVEnvCfg",
        "rl_games_cfg_entry_point": f"{config.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{config.__name__}.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
        "skrl_cfg_entry_point": f"{config.__name__}:skrl_ppo_cfg.yaml",
        "sb3_cfg_entry_point": f"{config.__name__}:sb3_ppo_cfg.yaml",
    },
)

gym.register(
    id="Isaac-Z-Rotation-Aerotaxi-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.isaac_z_rotation_aerotaxi_v0:UAVEnvCfg",
        "rl_games_cfg_entry_point": f"{config.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{config.__name__}.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
        "skrl_cfg_entry_point": f"{config.__name__}:skrl_ppo_cfg.yaml",
        "sb3_cfg_entry_point": f"{config.__name__}:sb3_ppo_cfg.yaml",
    },
)

gym.register(
    id="Isaac-Command-Aerotaxi-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.isaac_command_aerotaxi_v0:UAVEnvCfg",
        "rl_games_cfg_entry_point": f"{config.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{config.__name__}.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
        "skrl_cfg_entry_point": f"{config.__name__}:skrl_ppo_cfg.yaml",
        "sb3_cfg_entry_point": f"{config.__name__}:sb3_ppo_cfg.yaml",
    },
)

gym.register(
    id="Isaac-Command-Aerotaxi-v1",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.isaac_command_aerotaxi_v1:UAVEnvCfg",
        "rl_games_cfg_entry_point": f"{config.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{config.__name__}.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
        "skrl_cfg_entry_point": f"{config.__name__}:skrl_ppo_cfg.yaml",
        "sb3_cfg_entry_point": f"{config.__name__}:sb3_ppo_cfg.yaml",
    },
)

gym.register(
    id="Isaac-Command-Aerotaxi-v2",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.isaac_command_aerotaxi_v2:UAVEnvCfg",
        "rl_games_cfg_entry_point": f"{config.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{config.__name__}.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
        "skrl_cfg_entry_point": f"{config.__name__}:skrl_ppo_cfg.yaml",
        "sb3_cfg_entry_point": f"{config.__name__}:sb3_ppo_cfg.yaml",
    },
)