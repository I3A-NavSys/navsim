from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class Command2PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 100
    max_iterations = 4001
    save_interval = 100
    experiment_name = "command2"
    run_name = "ppo_random_command2_28"
    resume = False
    empirical_normalization = False
    csv_path_metrics = "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v2"
    callbacks = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.0,
        actor_hidden_dims=[128, 64],
        critic_hidden_dims=[64, 64],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=0.5,
        use_clipped_value_loss=True,
        clip_param=0.1,
        entropy_coef=0.0,
        num_learning_epochs=4,
        num_mini_batches=2,
        learning_rate=0.0001,
        schedule="adaptive",
        gamma=0.98,
        lam=0.95,
        desired_kl=0.02,
        max_grad_norm=2.0,
    )