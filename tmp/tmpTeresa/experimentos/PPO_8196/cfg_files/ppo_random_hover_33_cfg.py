from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class HoverPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 24
    max_iterations = 1501
    save_interval = 100
    experiment_name = "hover"
    run_name = "ppo_random_hover_33"
    resume = False
    empirical_normalization = False
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.2,
        actor_hidden_dims=[64, 64],
        critic_hidden_dims=[64, 64],
        activation="gelu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.0,
        num_learning_epochs=4,
        num_mini_batches=8,
        learning_rate=0.001,
        schedule="adaptive",
        gamma=0.98,
        lam=0.97,
        desired_kl=0.01,
        max_grad_norm=1.0,
    )