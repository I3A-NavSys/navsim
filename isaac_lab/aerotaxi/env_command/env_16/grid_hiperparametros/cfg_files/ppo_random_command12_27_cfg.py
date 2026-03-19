from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class Command12PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 128 
    max_iterations = 2001
    save_interval = 100
    experiment_name = "command12"
    run_name = "ppo_random_command12_27"
    resume = False
    empirical_normalization = True
    csv_path_metrics = "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v12"
    callbacks = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.8,
        actor_hidden_dims=[128, 128],
        critic_hidden_dims=[128, 128],
        activation="relu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=2.0,
        use_clipped_value_loss=True,
        clip_param=0.1,
        entropy_coef=0.0,
        num_learning_epochs=12,
        num_mini_batches=4,
        learning_rate=0.001,
        schedule="adaptive",
        gamma=0.98,
        lam=0.97,
        desired_kl=0.01,
        max_grad_norm=0.5,
    )