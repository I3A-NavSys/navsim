from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class HoverPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 2001
    save_interval = 100
    experiment_name = "hover"
    run_name = "ppo_random_hover_87"
    resume = False
    empirical_normalization = False
    csv_path_metrics = "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_v2"
    callbacks = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.0,
        actor_hidden_dims=[256, 256],
        critic_hidden_dims=[256, 256],
        activation="relu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=2.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.05,
        num_learning_epochs=8,
        num_mini_batches=2,
        learning_rate=0.001,
        schedule="adaptive",
        gamma=0.99,
        lam=0.9,
        desired_kl=0.01,
        max_grad_norm=0.5,
    )