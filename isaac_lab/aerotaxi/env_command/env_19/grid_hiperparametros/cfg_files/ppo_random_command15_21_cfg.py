from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class Command15PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    seed = 42
    num_steps_per_env = 128
    max_iterations = 4000
    save_interval = 100
    experiment_name = "command15"
    run_name = "ppo_random_command15_21"
    resume = False
    empirical_normalization = True
    csv_path_metrics = "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v19"
    callbacks = True
    early_stopping_patience = 500      
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.1,
        actor_hidden_dims=[256, 256],
        critic_hidden_dims=[128, 128],
        activation="gelu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=2.0,
        use_clipped_value_loss=True,
        clip_param=0.1,
        entropy_coef=0.0,
        num_learning_epochs=12,
        num_mini_batches=2,
        learning_rate=5e-05,
        schedule="adaptive",
        gamma=0.98,
        lam=0.9,
        desired_kl=0.008,
        max_grad_norm=1.0,
    )
