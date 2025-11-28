from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class HoverPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 1001
    save_interval = 100
    experiment_name = "hover"
    run_name = "ppo_random_hover_6"
    resume = False
    empirical_normalization = False
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.0,
        actor_hidden_dims=[256, 256],
        critic_hidden_dims=[256, 256],
        activation="gelu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.1,
        entropy_coef=0.05,
        num_learning_epochs=8,
        num_mini_batches=4,
        learning_rate=0.003,
        schedule="adaptive",
        gamma=0.99,
        lam=0.9,
        desired_kl=0.01,
        max_grad_norm=0.5,
    )