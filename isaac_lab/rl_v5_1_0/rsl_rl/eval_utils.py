"""Evaluación de políticas: UN episodio completo por entorno, vectorizada.

Sustituye a la rama `else:` de play.py (bucle de 1000 pasos con contador propio).

Garantías
---------
* Cada entorno aporta exactamente un episodio: o termina por fallo (se anota la causa) o llega a time-out (1250 pasos).
* Nunca se registran estados posteriores al reset (en la versión anterior el mismo env_id mezclaba episodios).
* Posición del dron = centro de masas (root_com_pos_w), la misma magnitud que usan reward y target.
* success = time-out sin terminación por fallo. Todas las tasas son por episodio, no snapshots de `info["log"]`.
"""
from __future__ import annotations

import os
import numpy as np
import pandas as pd
import torch

TERMS = ("below_min_altitude", "bad_attitude", "safety_shutdown")
OVERRIDE_VARS = ("AEROTAXI_TAU_MS", "AEROTAXI_WIND_MS", "AEROTAXI_GUST_STD", "AEROTAXI_TRAJ", "HORIZON_MODE")


@torch.inference_mode()
def evaluate_first_episode(env, policy, obs, *, max_steps: int, record_states: bool = False, asset_name: str = "aerotaxi",
                           command_name: str = "vel_command"):
    uenv = env.unwrapped
    dev, n = uenv.device, uenv.num_envs
    asset = uenv.scene[asset_name]
    cmd = uenv.command_manager.get_term(command_name)
    origins = uenv.scene.env_origins
    dt = uenv.step_dt
    rterms = list(uenv.reward_manager.active_terms)

    alive = torch.ones(n, dtype=torch.bool, device=dev)
    failed = torch.zeros(n, dtype=torch.bool, device=dev)
    cause = torch.full((n,), -1, dtype=torch.long, device=dev)
    steps = torch.zeros(n, dtype=torch.long, device=dev)
    ret = torch.zeros(n, device=dev)
    term_sum = torch.zeros(n, len(rterms), device=dev)
    frames, masks = [], []
    prev_a = None
    ar_sum = torch.zeros(n, device=dev)       # suma de ||a_t - a_{t-1}|| mientras el episodio vive
    ar_cnt = torch.zeros(n, device=dev)
    sat_sum = torch.zeros(n, device=dev)      # pasos con algún rotor saturado (|a| >= 0.99)

    for t in range(max_steps):
        actions = policy(obs)
        w = alive.float()                                  # entornos vivos ANTES de este paso
        if actions is not None:
            a = actions.clamp(-1.0, 1.0)                   # lo que realmente aplica el entorno
            if prev_a is not None:
                ar_sum += (a - prev_a).norm(dim=1) * w
                ar_cnt += w
            sat_sum += (a.abs() >= 0.99).any(dim=1).float() * w
            prev_a = a
        obs, rew, done, info = env.step(actions)
        done = done.bool()
        # --- estado POST-step: válido solo para entornos vivos que NO han terminado en este paso ---
        valid = alive & ~done
        if record_states:
            pos = asset.data.root_com_pos_w - origins
            frames.append(torch.cat([pos, cmd.target_pos, asset.data.root_com_lin_vel_w, cmd.target_vel], dim=1).float().cpu())
            masks.append(valid.cpu())
        # --- recompensas (el paso terminal SÍ cuenta: se calcula antes del reset) ---
        ret += rew.reshape(-1) * w
        step_r = getattr(uenv.reward_manager, "_step_reward", None)       # (n, n_terms), ponderada, en unidades/seg
        if step_r is not None:
            term_sum += step_r * dt * w.unsqueeze(1)
        # --- final de episodio ---
        fin = alive & done
        if fin.any():
            to = info.get("time_outs", torch.zeros_like(done)).bool()
            fail_now = fin & ~to
            failed |= fail_now
            steps[fin] = t + 1
            for i, name in enumerate(TERMS):
                try:
                    flag = uenv.termination_manager.get_term(name).bool()
                except Exception:
                    continue
                sel = fail_now & flag & (cause < 0)
                cause[sel] = i
            alive &= ~done
        if not alive.any():
            break

    steps[alive] = max_steps                      # (no debería ocurrir si max_steps > max_episode_length)
    out = {
        "summary": pd.DataFrame({
            "env_id": np.arange(n), "T": steps.cpu().numpy(), "failed": failed.cpu().numpy(),
            "cause": [TERMS[c] if c >= 0 else ("timeout" if not f else "other")
                      for c, f in zip(cause.cpu().tolist(), failed.cpu().tolist())],
            "return": ret.cpu().numpy(),
            "action_rate": (ar_sum / ar_cnt.clamp(min=1.0)).cpu().numpy(),          # media de ||Δa|| por paso (suavidad)
            "sat_frac": (sat_sum / steps.clamp(min=1).float()).cpu().numpy(),        # fracción de pasos con saturación
            **{f"ret_{name}": (term_sum[:, i] / uenv.max_episode_length_s).cpu().numpy() for i, name in enumerate(rterms)},
        }),
        "traj": None, "dt": dt,
    }
    if record_states and frames:
        arr = torch.stack(frames).numpy()                  # (T, n, 12)
        msk = torch.stack(masks).numpy()                   # (T, n)
        tt, ee = np.nonzero(msk)
        a = arr[tt, ee]
        df = pd.DataFrame({
            "env_id": ee, "episode": 0, "step": tt,
            "drone_x": a[:, 0], "drone_y": a[:, 1], "drone_z": a[:, 2],
            "target_x": a[:, 3], "target_y": a[:, 4], "target_z": a[:, 5],
            "drone_vel_x": a[:, 6], "drone_vel_y": a[:, 7], "drone_vel_z": a[:, 8],
            "target_vel_x": a[:, 9], "target_vel_y": a[:, 10], "target_vel_z": a[:, 11],
        })
        fail_by_env = out["summary"].set_index("env_id")["failed"]
        last = df.groupby("env_id")["step"].transform("max")
        df["terminated"] = (df["step"] == last) & df["env_id"].map(fail_by_env).values
        out["traj"] = df
    return out


def append_play_row(res: dict, run_name: str, num_envs: int, checkpoint, csv_path: str, tag: str = "", seed_eval=None,
                    extra: dict | None = None):
    """Una fila por evaluación, con las mismas columnas que usaba el notebook (reward, std, mean_Episode_*)
    más las condiciones de evaluación leídas de las variables de entorno: ya no hay que leer el CSV por posición."""
    s = res["summary"]
    n = len(s)
    row = {"id_run_name": run_name, "reward": s["return"].mean(), "max_reward": s["return"].max(), "std": s["return"].std(ddof=0),
           "num_envs_test": num_envs, "checkpoint": checkpoint, "tag": tag, "seed_eval": seed_eval,
           "n_episodes": n, "success": float((~s.failed).mean()),
           "mean_Episode_Termination/time_out": float((s.cause == "timeout").mean())}
    for name in TERMS:
        row[f"mean_Episode_Termination/{name}"] = float((s.cause == name).mean())
    row["action_rate"] = float(s["action_rate"].mean())
    row["sat_frac"] = float(s["sat_frac"].mean())
    for c in [c for c in s.columns if c.startswith("ret_")]:
        row[f"mean_Episode_Reward/{c[4:]}"] = float(s[c].mean())
    for k in OVERRIDE_VARS:
        row[k.lower()] = os.environ.get(k, "")
    row.update(extra or {})
    new = pd.DataFrame([row])
    if os.path.exists(csv_path):
        new = pd.concat([pd.read_csv(csv_path), new], ignore_index=True)
    new.to_csv(csv_path, index=False)


def save_trajectories(res: dict, save_dir: str, filename: str):
    if res["traj"] is None:
        return None
    os.makedirs(save_dir, exist_ok=True)
    p = os.path.join(save_dir, filename)
    res["traj"].to_csv(p, index=False)
    res["summary"].to_csv(os.path.join(save_dir, "episodes_summary.csv"), index=False)
    return p
