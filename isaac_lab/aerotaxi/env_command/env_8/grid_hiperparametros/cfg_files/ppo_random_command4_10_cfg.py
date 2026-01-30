from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class Command4PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 100
    max_iterations = 8001
    save_interval = 100
    experiment_name = "command4"
    run_name = "ppo_random_command4_10"
    resume = False
    empirical_normalization = False
    csv_path_metrics = "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v4"
    callbacks = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.1,
        actor_hidden_dims=[256, 256],
        critic_hidden_dims=[64, 64],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=0.5,
        use_clipped_value_loss=True,
        clip_param=0.1,
        entropy_coef=0.001,
        num_learning_epochs=8,
        num_mini_batches=2,
        learning_rate=0.003,
        schedule="adaptive",
        gamma=0.98,
        lam=0.95,
        desired_kl=0.005,
        max_grad_norm=0.5,
    )