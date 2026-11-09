# SPDX-License-Identifier: BSD-3-Clause
"""Test del brazo de palanca de los rotores (¿duplicado?). Colócalo junto a play.py (necesita cli_args y lever_arm_math).

Uso (desde isaaclab.bat -p):
  check_lever_arm.py --task Isaac-Command15-Aerotaxi-RANDOM-100 --num_envs 1 --headless
  con las variables de entorno  AEROTAXI_TAU_MS=0  AEROTAXI_WIND_MS=0  AEROTAXI_GUST_STD=0   (sin retardo ni viento)

Qué hace: sin aleatorización (sin inclinación, sin velocidad, masa nominal) deja el dron en hover, aplica un empuje conocido
en un solo rotor durante 0.2 s y compara la aceleración angular medida con la del modelo. Imprime líneas [LEVER].
"""
import argparse
import json
import sys
from pathlib import Path

isaaclab_path = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(isaaclab_path))

from isaaclab.app import AppLauncher

import cli_args  # isort: skip

parser = argparse.ArgumentParser(description="Test del brazo de palanca.")
parser.add_argument("--task", type=str, default=None)
parser.add_argument("--agent", type=str, default="rsl_rl_cfg_entry_point")
parser.add_argument("--num_envs", type=int, default=1)
parser.add_argument("--seed", type=int, default=None)
parser.add_argument("--n_apply", type=int, default=10, help="pasos de control con el empuje aplicado (0.02 s cada uno)")
cli_args.add_rsl_rl_args(parser)
AppLauncher.add_app_launcher_args(parser)
args_cli, hydra_args = parser.parse_known_args()
sys.argv = [sys.argv[0]] + hydra_args
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""
import gymnasium as gym
import numpy as np
import torch

import isaaclab_tasks  # noqa: F401
import aerotaxi  # noqa: F401
from isaaclab_tasks.utils.hydra import hydra_task_config

from lever_arm_math import PATTERNS, fit_alpha, predict_alphas, verdict


@hydra_task_config(args_cli.task, args_cli.agent)
def main(env_cfg, agent_cfg):
    agent_cfg = cli_args.update_rsl_rl_cfg(agent_cfg, args_cli)
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.seed = agent_cfg.seed

    # Condiciones limpias: pose fija a 10 m sobre el estado por defecto (evita below_min_altitude), sin velocidad, masa nominal
    ev = env_cfg.events
    ev.reset_pos.params["pose_range"] = {"x": (0.0, 0.0), "y": (0.0, 0.0), "z": (10.0, 10.0),
                                         "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0)}
    ev.reset_pos.params["velocity_range"] = {"x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0)}
    if getattr(ev, "randomize_mass", None) is not None:
        ev.randomize_mass.params["mass_distribution_params"] = (1.0, 1.0)

    env = gym.make(args_cli.task, cfg=env_cfg)
    uenv = env.unwrapped
    asset = uenv.scene["aerotaxi"]
    dev, dt = uenv.device, uenv.step_dt
    mass = float(asset.data.default_mass[0].sum())
    I = asset.data.default_inertia[0, 0].cpu().numpy()          # (Ixx, Ixy, Ixz, Iyx, Iyy, Iyz, Izx, Izy, Izz)
    inertia = (float(I[0]), float(I[4]), float(I[8]))
    print(f"[LEVER] masa = {mass:.1f} kg | inercia (Ixx, Iyy, Izz) = {tuple(round(v, 1) for v in inertia)} | dt = {dt}", flush=True)

    zeros = torch.zeros(uenv.num_envs, 4, device=dev)
    jpos0 = asset.data.default_joint_pos.clone()

    def clean_reset():
        """reset + articulaciones de los rotores a cero (los links giran libres: su estado NO se reinicia con el env)."""
        env.reset()
        asset.write_joint_state_to_sim(jpos0, torch.zeros_like(asset.data.joint_vel))
        for _ in range(5):                                      # calentamiento en hover (a = 0)
            env.step(zeros)

    def record(act_t, n):
        rec = []
        for _ in range(n):
            env.step(act_t)
            rec.append(asset.data.root_com_ang_vel_b[0].cpu().numpy().copy())
        return np.array(rec)

    measured, predicted, drifts = {}, {}, []
    for name, act in PATTERNS.items():
        clean_reset()
        w0 = asset.data.root_com_ang_vel_b[0].cpu().numpy()
        a = torch.tensor(act, device=dev, dtype=torch.float32).repeat(uenv.num_envs, 1)
        base = fit_alpha(record(zeros, args_cli.n_apply), dt)    # fase 1: hover (a=0) -> deriva propia del hover
        patt = fit_alpha(record(a, args_cli.n_apply), dt)        # fase 2: patrón (mismo episodio)
        drifts.append(base)
        measured[name] = patt - base                             # respuesta NETA al empuje añadido
        predicted[name] = predict_alphas(act, inertia, mass, 1.0)
        print(f"[LEVER] {name}: |omega| antes = {np.abs(w0).max():.4f} rad/s | deriva a=0 = {np.round(base, 3)} | "
              f"alpha neta = {np.round(measured[name], 3)} | modelo 1x = {np.round(predicted[name], 3)}", flush=True)
    drift = np.mean(np.array(drifts), axis=0)
    print(f"[LEVER] deriva media en hover (a=0) = {np.round(drift, 3)} rad/s^2 (con un modelo equilibrado debe ser ~0)", flush=True)

    texto, rows = verdict(measured, predicted, drift=drift)
    print("[LEVER] ---------------- ratio = alpha_medida / alpha_modelo(1x) ----------------", flush=True)
    for (name, lab, p, m, r, c) in rows:
        print(f"[LEVER] {name:<28} {lab:<5} modelo={p:+.3f}  medida={m:+.3f}  ratio={r:5.2f}  -> {c}", flush=True)
    print(f"[LEVER] VEREDICTO: {texto}", flush=True)
    out = Path(__file__).with_name("lever_arm_result.json")
    out.write_text(json.dumps({"veredicto": texto, "filas": [[n, l, float(p), float(m), float(r), c] for n, l, p, m, r, c in rows],
                               "inercia": inertia, "masa": mass, "deriva_hover": [float(v) for v in drift]}, indent=1), encoding="utf-8")
    env.close()


if __name__ == "__main__":
    main()
    simulation_app.close()
