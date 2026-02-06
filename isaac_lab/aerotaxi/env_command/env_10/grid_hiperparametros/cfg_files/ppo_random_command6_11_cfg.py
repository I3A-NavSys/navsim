from isaaclab.utils import configclass
from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

@configclass
class Command6PPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 625
    max_iterations = 4001
    save_interval = 100
    experiment_name = "command6"
    run_name = "ppo_random_command6_11"
    resume = False
    empirical_normalization = True
    csv_path_metrics = "C:/Users/Teresa/Documents/GitHub/navsim/tmp/tmpTeresa/resultados_grid_command_v6"
    callbacks = True
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.5,
        actor_hidden_dims=[128, 128],
        critic_hidden_dims=[128, 128],
        activation="elu",
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=1.0,
        use_clipped_value_loss=True,
        clip_param=0.2,
        entropy_coef=0.01,           
        num_learning_epochs=5,       
        num_mini_batches=80,         
        learning_rate=5e-5,         
        schedule="adaptive",         
        gamma=0.99,                  
        lam=0.95,                    
        desired_kl=0.01,             
        max_grad_norm=1.0,
    )