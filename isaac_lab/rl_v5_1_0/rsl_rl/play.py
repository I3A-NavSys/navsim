# Copyright (c) 2022-2025, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Script to play a checkpoint if an RL agent from RSL-RL."""

"""Launch Isaac Sim Simulator first."""

import argparse
import sys
from pathlib import Path

isaaclab_path = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(isaaclab_path))

from isaaclab.app import AppLauncher
from MyOnPolicyRunner import MyOnPolicyRunner

# local imports
import cli_args  # isort: skip

# add argparse arguments
parser = argparse.ArgumentParser(description="Train an RL agent with RSL-RL.")
parser.add_argument("--video", action="store_true", default=False, help="Record videos during training.")
parser.add_argument("--video_length", type=int, default=200, help="Length of the recorded video (in steps).")
parser.add_argument(
    "--disable_fabric", action="store_true", default=False, help="Disable fabric and use USD I/O operations."
)
parser.add_argument("--num_envs", type=int, default=None, help="Number of environments to simulate.")
parser.add_argument("--task", type=str, default=None, help="Name of the task.")
parser.add_argument(
    "--agent", type=str, default="rsl_rl_cfg_entry_point", help="Name of the RL agent configuration entry point."
)
parser.add_argument("--seed", type=int, default=None, help="Seed used for the environment")
parser.add_argument(
    "--use_pretrained_checkpoint",
    action="store_true",
    help="Use the pre-trained checkpoint from Nucleus.",
)
parser.add_argument("--real-time", action="store_true", default=False, help="Run in real-time, if possible.")

# ---- Teresa -----
parser.add_argument("--see", action="store_true", default=False, help="Disable the option of stopping at the first episodes of evaluation")
# -----------------


# append RSL-RL cli arguments
cli_args.add_rsl_rl_args(parser)
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli, hydra_args = parser.parse_known_args()
# always enable cameras to record video
if args_cli.video:
    args_cli.enable_cameras = True

# clear out sys.argv for Hydra
sys.argv = [sys.argv[0]] + hydra_args

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym
import os
import time
import torch

from rsl_rl.runners import DistillationRunner, OnPolicyRunner

from isaaclab.envs import (
    DirectMARLEnv,
    DirectMARLEnvCfg,
    DirectRLEnvCfg,
    ManagerBasedRLEnvCfg,
    multi_agent_to_single_agent,
)
from isaaclab.utils.assets import retrieve_file_path
from isaaclab.utils.dict import print_dict
from isaaclab.utils.pretrained_checkpoint import get_published_pretrained_checkpoint

from isaaclab_rl.rsl_rl import RslRlBaseRunnerCfg, RslRlVecEnvWrapper, export_policy_as_jit, export_policy_as_onnx

import isaaclab_tasks  # noqa: F401
import aerotaxi
from isaaclab_tasks.utils import get_checkpoint_path
from isaaclab_tasks.utils.hydra import hydra_task_config

# PLACEHOLDER: Extension template (do not remove this comment)


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg: ManagerBasedRLEnvCfg | DirectRLEnvCfg | DirectMARLEnvCfg, agent_cfg: RslRlBaseRunnerCfg):
    """Play with RSL-RL agent."""
    # grab task name for checkpoint path
    task_name = args_cli.task.split(":")[-1]
    train_task_name = task_name.replace("-Play", "")

    # override configurations with non-hydra CLI arguments
    agent_cfg: RslRlBaseRunnerCfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs if args_cli.num_envs is not None else env_cfg.scene.num_envs

    # set the environment seed
    # note: certain randomizations occur in the environment initialization so we set the seed here
    env_cfg.seed = agent_cfg.seed
    env_cfg.sim.device = args_cli.device if args_cli.device is not None else env_cfg.sim.device

    # specify directory for logging experiments
    log_root_path = os.path.join("logs", "rsl_rl", agent_cfg.experiment_name)
    log_root_path = os.path.abspath(log_root_path)
    print(f"[INFO] Loading experiment from directory: {log_root_path}")
    if args_cli.use_pretrained_checkpoint:
        resume_path = get_published_pretrained_checkpoint("rsl_rl", train_task_name)
        if not resume_path:
            print("[INFO] Unfortunately a pre-trained checkpoint is currently unavailable for this task.")
            return
    elif args_cli.checkpoint:
        resume_path = retrieve_file_path(args_cli.checkpoint)
    else:
        resume_path = get_checkpoint_path(log_root_path, agent_cfg.load_run, agent_cfg.load_checkpoint)

    log_dir = os.path.dirname(resume_path)

    # set the log directory for the environment (works for all environment types)
    env_cfg.log_dir = log_dir
    # ---- Teresa ------
    env_cfg.is_test_mode = True
    # ------------------
    # create isaac environment
    env = gym.make(args_cli.task, cfg=env_cfg, render_mode="rgb_array" if args_cli.video else None)

    # convert to single-agent instance if required by the RL algorithm
    if isinstance(env.unwrapped, DirectMARLEnv):
        env = multi_agent_to_single_agent(env)

    # wrap for video recording
    if args_cli.video:
        video_kwargs = {
            "video_folder": os.path.join(log_dir, "videos", "play"),
            "step_trigger": lambda step: step == 0,
            "video_length": args_cli.video_length,
            "disable_logger": True,
        }
        print("[INFO] Recording videos during training.")
        print_dict(video_kwargs, nesting=4)
        env = gym.wrappers.RecordVideo(env, **video_kwargs)

    # wrap around environment for rsl-rl
    env = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)

    print(f"[INFO]: Loading model checkpoint from: {resume_path}")
    # load previously trained model
    if agent_cfg.class_name == "OnPolicyRunner":
        runner = MyOnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    elif agent_cfg.class_name == "DistillationRunner":
        runner = DistillationRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
    else:
        raise ValueError(f"Unsupported runner class: {agent_cfg.class_name}")
    runner.load(resume_path)

    # obtain the trained policy for inference
    policy = runner.get_inference_policy(device=env.unwrapped.device)

    # extract the neural network module
    # we do this in a try-except to maintain backwards compatibility.
    try:
        # version 2.3 onwards
        policy_nn = runner.alg.policy
    except AttributeError:
        # version 2.2 and below
        policy_nn = runner.alg.actor_critic

    # extract the normalizer
    if hasattr(policy_nn, "actor_obs_normalizer"):
        normalizer = policy_nn.actor_obs_normalizer
    elif hasattr(policy_nn, "student_obs_normalizer"):
        normalizer = policy_nn.student_obs_normalizer
    else:
        normalizer = None

    # export policy to onnx/jit
    export_model_dir = os.path.join(os.path.dirname(resume_path), "exported")
    export_policy_as_jit(policy_nn, normalizer=normalizer, path=export_model_dir, filename="policy.pt")
    export_policy_as_onnx(policy_nn, normalizer=normalizer, path=export_model_dir, filename="policy.onnx")

    dt = env.unwrapped.step_dt

    # reset environment
    obs = env.get_observations()
    timestep = 0
    # simulate environment

    # ------- Teresa ---------
    import numpy as np
    rewbuffer = np.zeros(env.num_envs)
    episode_rewards = []  # Buffer para guardar las recompensas por episodio
    episode_steps = []  # Guardará el número de pasos por episodio

    # Define el número máximo de pasos por episodio
    max_episode_steps = 1000  # o cualquier número de pasos que consideres adecuado
    cur_episode_length = np.zeros(env.num_envs)
    # -------------------------

    if args_cli.see:
        while simulation_app.is_running():
            start_time = time.time()
            # run everything in inference mode
            with torch.inference_mode():
                # agent stepping
                actions = policy(obs)
                # env stepping

                # ------- Teresa ---------
                # obs, _, _, _ = env.step(actions) # original
                obs, rew, done, _ = env.step(actions)
                rew = rew.cpu().numpy().squeeze()
                done = done.cpu().numpy().squeeze()
                rewbuffer += rew
                cur_episode_length += 1
            
                # Termina los episodios cuando 'done' es 1 o se alcanza el máximo de pasos
                finished_ids = np.where((done > 0) | (cur_episode_length >= max_episode_steps))[0]
                
                # Por cada episodio que termine
                for idx in finished_ids:
                    # Guardar la recompensa acumulada del episodio
                    episode_rewards.append(rewbuffer[idx])  # Guardamos recompensa completa del episodio
                    episode_steps.append(cur_episode_length[idx])  # Guardamos el número de pasos del episodio
                    
                    # Reiniciar los valores para el siguiente episodio
                    rewbuffer[idx] = 0
                    cur_episode_length[idx] = 0
                    
                # Promedio de recompensa
                if len(episode_rewards) > 0:
                    average_reward = np.mean(episode_rewards)
                    max_reward = np.max(episode_rewards)
                    std = np.std(episode_rewards)
                else:
                    average_reward = 0
                    max_reward = 0
                    std = 0
                # -------------------------
            if args_cli.video:
                timestep += 1
                # Exit the play loop after recording one video
                if timestep == args_cli.video_length:
                    break

            # time delay for real-time evaluation
            sleep_time = dt - (time.time() - start_time)
            if args_cli.real_time and sleep_time > 0:
                time.sleep(sleep_time)
    else:
        ep_infos = []
        for step in range(max_episode_steps):
            start_time = time.time()
            # run everything in inference mode
            with torch.inference_mode():
                # agent stepping
                actions = policy(obs)
                # env stepping

                # ------- Teresa ---------
                # obs, _, _, _ = env.step(actions) # original
                obs, rew, done, info = env.step(actions)
                rew = rew.cpu().numpy().squeeze()
                done = done.cpu().numpy().squeeze()
                rewbuffer += rew
                cur_episode_length += 1
            
                # Termina los episodios cuando 'done' es 1 o se alcanza el máximo de pasos
                finished_ids = np.where((done > 0) | (cur_episode_length >= max_episode_steps))[0]
                
                # Por cada episodio que termine
                for idx in finished_ids:
                    # Guardar la recompensa acumulada del episodio
                    episode_rewards.append(rewbuffer[idx])
                    episode_steps.append(cur_episode_length[idx])

                    if "log" in info:
                        individual_rewards = {}
                        for key, value in info["log"].items():
                            if torch.is_tensor(value):
                                # Si el tensor tiene dimensiones (es un vector), indexamos por idx
                                if value.dim() > 0:
                                    val = value[idx].item()
                                # Si es un escalar (0-dim), tomamos el valor directamente
                                else:
                                    val = value.item()
                            else:
                                # Si es una lista o array de numpy con dimensiones
                                try:
                                    val = value[idx]
                                except (IndexError, TypeError):
                                    val = value
                            
                            individual_rewards[key] = val
                        ep_infos.append(individual_rewards)
                        
                # Promedio de recompensa
                if len(episode_rewards) > 0:
                    average_reward = np.mean(episode_rewards)
                    max_reward = np.max(episode_rewards)
                    std = np.std(episode_rewards)
                else:
                    average_reward = 0
                    max_reward = 0
                    std = 0
                # -------------------------
            if args_cli.video:
                timestep += 1
                # Exit the play loop after recording one video
                if timestep == args_cli.video_length:
                    break

            # time delay for real-time evaluation
            sleep_time = dt - (time.time() - start_time)
            if args_cli.real_time and sleep_time > 0:
                time.sleep(sleep_time)


        # --------- Teresa ----------------
        if runner.csv_path_metrics != None:
            import pandas as pd
            reward_terms_dict = {}
            if 'ep_infos' in locals() and len(ep_infos) > 0:
                keys = ep_infos[0].keys()
                for key in keys:
                    # Procesamos cada término asegurando que pase a CPU
                    reward_values = [
                        info[key].cpu().item() if torch.is_tensor(info[key]) else info[key] 
                        for info in ep_infos if key in info
                    ]
                    if reward_values:
                        reward_terms_dict[f"mean_{key}"] = [np.mean(reward_values)]

            # 3. Preparar el DataFrame con toda la información
            base_data = {
                'id_run_name': [runner.cfg['run_name']],
                'reward': [average_reward],
                'max_reward': [max_reward],
                'std': [std],
                'num_envs_test': [runner.env.num_envs],
                'checkpoint': [args_cli.checkpoint]
            }
            
            # Combinamos con los términos individuales (alive, pos_diff, etc.)
            base_data.update(reward_terms_dict)
            statistics_it = pd.DataFrame(base_data)

            # 4. Guardado en CSV
            csv_path = os.path.join(runner.csv_path_metrics, f"rewards_play_{type(runner.alg).__name__}.csv")
            
            if not os.path.exists(csv_path):
                statistics_it.to_csv(csv_path, mode='w', header=True, index=False)
            else:
                # Si el archivo existe, lo leemos para asegurar que las columnas coincidan 
                # (por si un modelo tiene más términos que otro)
                existing_df = pd.read_csv(csv_path)
                combined_df = pd.concat([existing_df, statistics_it], ignore_index=True)
                combined_df.to_csv(csv_path, index=False)
            # ---------------------------------

    # close the simulator
    env.close()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
