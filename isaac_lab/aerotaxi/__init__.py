# from .aerotaxi_env_reg import *
import gymnasium as gym
from . import env_hover, env_z_rotation, env_command

gym.register(
    id="Isaac-Hover-Aerotaxi-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{env_hover.__name__}.env_0.env:UAVEnvCfg",
        "rsl_rl_cfg_entry_point": f"{env_hover.__name__}.env_0.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Hover-Aerotaxi-v1",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{env_hover.__name__}.env_1.env:UAVEnvCfg",
        "rsl_rl_cfg_entry_point": f"{env_hover.__name__}.env_1.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Hover-Aerotaxi-v2",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{env_hover.__name__}.env_2.env:UAVEnvCfg",
        "rsl_rl_cfg_entry_point": f"{env_hover.__name__}.env_2.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
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
    id="Isaac-Z-Rotation-Aerotaxi-v1",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{env_z_rotation.__name__}.env_1.env:UAVEnvCfg",
        "rsl_rl_cfg_entry_point": f"{env_z_rotation.__name__}.env_1.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
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

gym.register(
    id="Isaac-Command-Aerotaxi-v3",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{env_command.__name__}.env_3.env:UAVEnvCfg",
        "rsl_rl_cfg_entry_point": f"{env_command.__name__}.env_3.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
    },
)

gym.register(
    id="Isaac-Command-Aerotaxi-v4",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{env_command.__name__}.env_4.env:UAVEnvCfg",
        "rsl_rl_cfg_entry_point": f"{env_command.__name__}.env_4.rsl_rl_ppo_cfg:AerotaxiPPORunnerCfg",
    },
)

for i in range(40):
    gym.register(
        id=f"Isaac-Hover-Aerotaxi-RANDOM-{i+1}",
        entry_point="isaaclab.envs:ManagerBasedRLEnv",
        disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": f"{env_hover.__name__}.env_2.env:UAVEnvCfg",
            "rsl_rl_cfg_entry_point": f"{env_hover.__name__}.env_2.grid_hiperparametros.cfg_files.ppo_random_hover_{i+1}_cfg:HoverPPORunnerCfg",
        },
    )
