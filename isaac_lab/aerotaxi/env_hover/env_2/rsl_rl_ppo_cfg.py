# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

from isaaclab.utils import configclass

from isaaclab_rl.rsl_rl import RslRlOnPolicyRunnerCfg, RslRlPpoActorCriticCfg, RslRlPpoAlgorithmCfg

# Algoritmo 1: PPO
@configclass
class AerotaxiPPORunnerCfg(RslRlOnPolicyRunnerCfg):
    num_steps_per_env = 128 
    max_iterations = 2500
    save_interval = 100
    experiment_name = "aerotaxi"
    empirical_normalization = True  # True mejor, dependiendo de si las acciones están normalizadas o no (rangos)
    clip_actions= 1.0 #clip para la normalización 0,1
    policy = RslRlPpoActorCriticCfg(
        init_noise_std=0.3, # cambiar a 0.3 -> es como el epsilon, con 1 da demasiada importancia a la exploración y 
                            # poca a la explotación (demasiado aleatorio, tarda en converger)
        actor_hidden_dims=[128, 128], # para 8 observaciones está bien. Si hubiera más, habría que poner más capas.
        critic_hidden_dims=[128, 256],
        activation="elu", # está correcto, ReLU es más rápido, pero podría ocurrir el problema de la neurona a 0 con
                          # entradas negativas (preguntar rangos)
        #layer_norm = True # Dropout
        #noise_decay_rate = 0.99 # epsilon con decay (primero explora luego explota)
    )
    algorithm = RslRlPpoAlgorithmCfg(
        value_loss_coef=0.8, # si veo que tarda en mejorar la política, le puedo bajar a 0.8 pero a priori está bien
        use_clipped_value_loss=True,
        clip_param=0.2, # valor por defecto 
        entropy_coef=0.02, 
        num_learning_epochs=5, 
        num_mini_batches=4,
        learning_rate=5e-4, #5e-4 ? quizas ese alpha es muy alto
        schedule="linear", 
        gamma=0.99,
        lam=0.95, 
        desired_kl=0.01, # nivel objetivo de divergencia
        max_grad_norm=1.0, #limita la magnitud del gradiente
    )
