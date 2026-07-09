from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class Command13PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 128 
    max_iterations = 4001
    save_interval = 100
    experiment_name = "command13"
    run_name = "ppo_random_command13_23"
    resume = False
    empirical_normalization = True
    csv_path_metrics = "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v13"
    callbacks = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.2,
        actor_hidden_dims=[128, 128],
        critic_hidden_dims=[64, 64],
        activation="relu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.3,
        entropy_coef=0.05,
        num_learning_epochs=12,
        num_mini_batches=4,
        learning_rate=5.0e-05,
        schedule="adaptive",
        gamma=0.95,
        lam=0.95,
        desired_kl=0.02,
        max_grad_norm=0.5,
    )