from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class Command8PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 100 
    max_iterations = 1001
    save_interval = 100
    experiment_name = "command8"
    run_name = "ppo_random_command8_19"
    resume = False
    empirical_normalization = False
    csv_path_metrics = "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v8"
    callbacks = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.1,
        actor_hidden_dims=[64, 64],
        critic_hidden_dims=[64, 64],
        activation="tanh",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=2.0,
        use_clipped_value_loss=True,
        clip_param=0.3,
        entropy_coef=0.001,
        num_learning_epochs=4,
        num_mini_batches=2,
        learning_rate=0.003,
        schedule="adaptive",
        gamma=0.99,
        lam=0.9,
        desired_kl=0.01,
        max_grad_norm=0.5,
    )