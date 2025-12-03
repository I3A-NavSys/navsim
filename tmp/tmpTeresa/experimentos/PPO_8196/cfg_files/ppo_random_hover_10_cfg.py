from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class HoverPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 1501
    save_interval = 100
    experiment_name = "hover"
    run_name = "ppo_random_hover_10"
    resume = False
    empirical_normalization = False
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.5,
        actor_hidden_dims=[128, 128],
        critic_hidden_dims=[128, 128],
        activation="gelu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=0.5,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.05,
        num_learning_epochs=12,
        num_mini_batches=2,
        learning_rate=0.001,
        schedule="adaptive",
        gamma=0.98,
        lam=0.97,
        desired_kl=0.01,
        max_grad_norm=2.0,
    )