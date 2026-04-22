from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class Command14PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 128 
    max_iterations = 4001
    save_interval = 100
    experiment_name = "command14"
    run_name = "ppo_random_command14_5"
    resume = False
    empirical_normalization = True
    csv_path_metrics = "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v14"
    callbacks = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.8,
        actor_hidden_dims=[128, 64],
        critic_hidden_dims=[128, 128],
        activation="tanh",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=2.0,
        use_clipped_value_loss=True,
        clip_param=0.1,
        entropy_coef=0.0,
        num_learning_epochs=8,
        num_mini_batches=4,
        learning_rate=5e-05,
        schedule="adaptive",
        gamma=0.99,
        lam=0.95,
        desired_kl=0.02,
        max_grad_norm=0.5,
    )