from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class Command2PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 2001
    save_interval = 100
    experiment_name = "command2"
    run_name = "ppo_random_command2_29"
    resume = False
    empirical_normalization = False
    csv_path_metrics = "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v2"
    callbacks = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=1.0,
        actor_hidden_dims=[64, 64],
        critic_hidden_dims=[256, 256],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.0,
        num_learning_epochs=12,
        num_mini_batches=4,
        learning_rate=0.0003,
        schedule="adaptive",
        gamma=0.98,
        lam=0.9,
        desired_kl=0.005,
        max_grad_norm=2.0,
    )