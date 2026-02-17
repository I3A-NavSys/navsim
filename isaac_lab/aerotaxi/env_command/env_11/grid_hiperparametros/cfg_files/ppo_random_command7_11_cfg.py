from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg
import sys
sys.path.append('C:/Users/Teresa/Documents/GitHub/navsim/isaac_lab/rl_v5_1_0/rsl_rl')

@configclass
class Command7PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 512
    max_iterations = 4001
    save_interval = 100
    experiment_name = "command7"
    run_name = "ppo_random_command7_11"
    resume = False
    empirical_normalization = True
    csv_path_metrics = "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v7"
    callbacks = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.2,
        actor_hidden_dims=[64, 64],
        critic_hidden_dims=[128, 128],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=0.5,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,
        num_learning_epochs=4,
        num_mini_batches=8,
        learning_rate=0.0001,
        schedule="adaptive",
        gamma=0.99,
        lam=0.9,
        desired_kl=0.005,
        max_grad_norm=0.5,
    )