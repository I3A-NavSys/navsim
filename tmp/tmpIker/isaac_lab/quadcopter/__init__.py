import gymnasium as gym

gym.register(
    id="Isaac-Hover-Quadcopter-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.managed_env_rl:QuadcopterEnvCfg",
        "rsl_rl_cfg_entry_point": f"{__name__}.rsl_rl_ppo_cfg:QuadcopterPPORunnerCfg"
    }
)