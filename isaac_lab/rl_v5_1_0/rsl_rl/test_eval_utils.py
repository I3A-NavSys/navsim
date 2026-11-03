"""Prueba de eval_utils con un entorno simulado (CPU, sin Isaac)."""
import numpy as np, pandas as pd, torch
from eval_utils import evaluate_first_episode, append_play_row, save_trajectories
import tempfile, os


class _Obj: pass


class FakeEnv:
    def __init__(self, n=8, max_len=50, fails=None):
        self.unwrapped = self
        self.num_envs, self.device, self.step_dt = n, "cpu", 0.02
        self.max_episode_length, self.max_episode_length_s = max_len, max_len * 0.02
        self.fails = fails or {}
        self.t = torch.zeros(n, dtype=torch.long)
        self.fail_flag = {k: torch.zeros(n, dtype=torch.bool) for k in ("below_min_altitude", "bad_attitude", "safety_shutdown")}
        a = _Obj(); a.data = _Obj(); a.data.root_com_pos_w = torch.zeros(n, 3); a.data.root_com_lin_vel_w = torch.zeros(n, 3)
        self.scene = {"aerotaxi": a}; self.scene_origins = torch.zeros(n, 3)
        self.scene = _SceneDict(self.scene, self.scene_origins)
        c = _Obj(); c.target_pos = torch.ones(n, 3); c.target_vel = torch.zeros(n, 3)
        self.command_manager = _Obj(); self.command_manager.get_term = lambda name: c
        self.termination_manager = _Obj(); self.termination_manager.get_term = lambda name: self.fail_flag[name]
        self.reward_manager = _Obj(); self.reward_manager.active_terms = ["r_a", "r_b"]
        self.reward_manager._step_reward = torch.tensor([[1.0, 2.0]]).repeat(n, 1)

    def step(self, actions):
        self.t += 1
        fail = torch.zeros(self.num_envs, dtype=torch.bool)
        for k in self.fail_flag: self.fail_flag[k][:] = False
        for env, (step, name) in self.fails.items():
            if int(self.t[env]) == step:
                fail[env] = True; self.fail_flag[name][env] = True
        to = (self.t >= self.max_episode_length) & ~fail
        done = fail | to
        self.scene["aerotaxi"].data.root_com_pos_w[:, 0] = self.t.float()
        rew = torch.ones(self.num_envs)
        self.t[done] = 0                                           # reset -> estado post-reset
        self.scene["aerotaxi"].data.root_com_pos_w[done, 0] = 0.0
        return None, rew, done, {"time_outs": to, "log": {}}


class _SceneDict(dict):
    def __init__(self, d, o): super().__init__(d); self.env_origins = o


def test_first_episode_semantics():
    env = FakeEnv(n=8, max_len=50, fails={0: (10, "bad_attitude"), 1: (20, "below_min_altitude")})
    res = evaluate_first_episode(env, lambda o: None, None, max_steps=55, record_states=True)
    s, tr = res["summary"], res["traj"]
    assert len(s) == 8 and s.failed.sum() == 2
    assert s.loc[0, "cause"] == "bad_attitude" and s.loc[1, "cause"] == "below_min_altitude"
    assert (s.loc[2:, "cause"] == "timeout").all()
    assert s.loc[0, "T"] == 10 and s.loc[2, "T"] == 50
    assert s.loc[0, "return"] == 10 and s.loc[2, "return"] == 50           # el paso terminal cuenta en la recompensa
    # reward por término normalizado por max_episode_length_s: sum(r*dt)/25s -> 50 pasos * 1/s*dt / (50*dt) = 1.0
    assert abs(s.loc[2, "ret_r_a"] - 1.0) < 1e-6 and abs(s.loc[2, "ret_r_b"] - 2.0) < 1e-6
    # ningún estado posterior al fallo / al reset
    e0 = tr[tr.env_id == 0]
    assert e0.step.max() == 8 and (e0.drone_x.values == e0.step.values + 1).all()   # x = t (post-step); sin ceros de reset
    assert e0.terminated.iloc[-1] and not e0.terminated.iloc[:-1].any()
    assert not tr[tr.env_id == 2].terminated.any()
    assert (tr.groupby("env_id").episode.nunique() == 1).all()


def test_csv_roundtrip():
    env = FakeEnv(n=4, max_len=30, fails={0: (5, "bad_attitude")})
    res = evaluate_first_episode(env, lambda o: None, None, max_steps=35, record_states=True)
    with tempfile.TemporaryDirectory() as d:
        p = os.path.join(d, "r.csv")
        append_play_row(res, "ppo_x_1", 4, "ck.pt", p, tag="t1", seed_eval=1001)
        append_play_row(res, "ppo_x_1", 4, "ck.pt", p, tag="t2", seed_eval=1001)
        df = pd.read_csv(p)
        assert len(df) == 2 and df.success.iloc[0] == 0.75 and df["mean_Episode_Termination/time_out"].iloc[0] == 0.75
        assert df["mean_Episode_Termination/bad_attitude"].iloc[0] == 0.25
        assert save_trajectories(res, d, "tracking_3D_ppo_x_1.csv")
        # compatibilidad con episode_metrics del notebook
        import re
        src = open("celdas_env_19.py").read()
        ns = {"np": np, "pd": pd, "DT": 0.02, "BURN_IN_S": 1.0}
        exec(src[src.index("def episode_metrics"): src.index("def wilson")], ns)
        ep = ns["episode_metrics"](pd.read_csv(os.path.join(d, "tracking_3D_ppo_x_1.csv")))
        assert ep.failed.sum() == 1 and len(ep) == 4


if __name__ == "__main__":
    test_first_episode_semantics(); print("OK first_episode"); test_csv_roundtrip(); print("OK csv")
