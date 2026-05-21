from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class Command8PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 100 
    max_iterations = 1001
    save_interval = 100
    experiment_name = "command8"
    run_name = "ppo_random_command8_31"
    resume = False
    empirical_normalization = False
    csv_path_metrics = "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v8"
    callbacks = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.2,
        actor_hidden_dims=[128, 64],
        critic_hidden_dims=[64, 64],
        activation="relu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=2.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.0,
        num_learning_epochs=8,
        num_mini_batches=8,
        learning_rate=0.001,
        schedule="adaptive",
        gamma=0.95,
        lam=0.97,
        desired_kl=0.02,
        max_grad_norm=0.5,
    )