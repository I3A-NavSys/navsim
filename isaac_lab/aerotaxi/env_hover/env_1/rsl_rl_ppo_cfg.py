# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg


@configclass
class AerotaxiPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 256
    max_iterations = 5000
    save_interval = 100
    experiment_name = "aerotaxi"
    empirical_normalization = True
    clip_actions=10
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.5,
        actor_hidden_dims=[64, 128, 256],
        critic_hidden_dims=[128, 256, 512],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=0.5,
        use_clipped_value_loss=True,
        clip_param=0.1,
        entropy_coef=0.01,
        num_learning_epochs=8,
        num_mini_batches=8,
        learning_rate=3e-4,
        schedule="linear",
        gamma=0.99,
        lam=0.9,
        desired_kl=0.008,
        max_grad_norm=0.5,
    )
