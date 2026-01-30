from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class Command3PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 100
    max_iterations = 8001
    save_interval = 100
    experiment_name = "command3"
    run_name = "ppo_random_command3_93"
    resume = False
    empirical_normalization = False
    csv_path_metrics = "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v3"
    callbacks = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.5,
        actor_hidden_dims=[128, 64],
        critic_hidden_dims=[128, 128],
        activation="tanh",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=0.5,
        use_clipped_value_loss=True,
        clip_param=0.1,
        entropy_coef=0.01,
        num_learning_epochs=12,
        num_mini_batches=4,
        learning_rate=0.0001,
        schedule="adaptive",
        gamma=0.95,
        lam=0.9,
        desired_kl=0.005,
        max_grad_norm=0.5,
    )