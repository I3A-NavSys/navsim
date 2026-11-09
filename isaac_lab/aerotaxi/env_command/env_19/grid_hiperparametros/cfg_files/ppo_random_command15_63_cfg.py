from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class Command15PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    seed = 33
    num_steps_per_env = 128 
    max_iterations = 1500
    save_interval = 100
    experiment_name = "ppo_ral_63_windonly_seed33"
    run_name = "ppo_ral_63_windonly_seed33"
    resume = False
    empirical_normalization = True
    csv_path_metrics = "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_ral_v19"
    callbacks = True
    early_stopping_patience = 0
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.2,
        actor_hidden_dims=[64, 64],
        critic_hidden_dims=[64, 64],
        activation="gelu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=0.5,
        use_clipped_value_loss=True,
        clip_param=0.1,
        entropy_coef=0.001,
        num_learning_epochs=12,
        num_mini_batches=4,
        learning_rate=5e-05,
        schedule="adaptive",
        gamma=0.99,
        lam=0.97,
        desired_kl=0.005,
        max_grad_norm=1.0,
    )