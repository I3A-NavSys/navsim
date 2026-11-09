"""Utilidades puras (solo torch) para env_19. No dependen de Isaac Lab, de modo que se pueden
testear en CPU (ver test_dynamics_utils.py) y citar/publicar de forma independiente.

Contenido
---------
rotor_wrench       : fuerza y momento netos de los empujes de rotor respecto al CoM (todo se aplica sobre la base).
ReferenceGenerator : punto guia con aceleracion acotada, valla con signo correcto y familias
                     analiticas (circle / lemniscate) para evaluacion fuera de distribucion.
ActuatorLag        : filtro de primer orden por entorno (constante de tiempo tau), con reset por episodio.
OUWind             : viento medio por episodio + rafagas Ornstein-Uhlenbeck, en ejes mundo.
"""
from __future__ import annotations

import math
import torch


def _u(n: int, lo: float, hi: float, device) -> torch.Tensor:
    return lo + (hi - lo) * torch.rand(n, device=device)


# --------------------------------------------------------------------------------------
# Actuadores
# --------------------------------------------------------------------------------------
class ActuatorLag:
    """Retardo de primer orden:  y_k = a*y_{k-1} + (1-a)*u_k,  a = exp(-dt/tau)  (ZOH exacto).

    * tau es "constante de tiempo del actuador" (NO un retardo de transporte puro).
    * tau se muestrea por entorno y por episodio en `reset`.
    * El estado se reinicia en cada episodio (el codigo anterior arrastraba el estado entre episodios).
    """

    def __init__(self, num_envs: int, dim: int, dt: float, device, tau_range_s=(0.0, 0.0)):
        self.dt, self.device, self.tau_range_s = dt, device, tau_range_s
        self.tau = torch.zeros(num_envs, 1, device=device)
        self.state = torch.zeros(num_envs, dim, device=device)
        self._initialised = False

    def reset(self, env_ids, y0: torch.Tensor):
        """y0: valor inicial del filtro para env_ids (p.ej. velocidad de hover)."""
        ids = slice(None) if env_ids is None else env_ids
        n = self.tau[ids].shape[0]
        lo, hi = self.tau_range_s
        self.tau[ids] = _u(n, lo, hi, self.device).unsqueeze(1)
        self.state[ids] = y0
        self._initialised = True

    def step(self, u: torch.Tensor) -> torch.Tensor:
        if not self._initialised:          # primer paso antes de ningun reset
            self.state[:] = u
            self._initialised = True
        a = torch.where(self.tau > 1e-6, torch.exp(-self.dt / self.tau.clamp(min=1e-6)), torch.zeros_like(self.tau))
        self.state = a * self.state + (1.0 - a) * u
        return self.state


# --------------------------------------------------------------------------------------
# Viento
# --------------------------------------------------------------------------------------
class OUWind:
    """Viento en ejes mundo = media por episodio (modulo U(speed_range), direccion horizontal uniforme)
    + rafagas OU (std `gust_std`, escala temporal `gust_tau_s`). Con speed_range=(0,0) y gust_std=0 no hay viento."""

    def __init__(self, num_envs: int, dt: float, device, speed_range=(0.0, 0.0), gust_std=0.0, gust_tau_s=3.0):
        self.dt, self.device = dt, device
        self.speed_range, self.gust_std, self.gust_tau_s = speed_range, gust_std, gust_tau_s
        self.mean = torch.zeros(num_envs, 3, device=device)
        self.gust = torch.zeros(num_envs, 3, device=device)

    @property
    def enabled(self) -> bool:
        return self.speed_range[1] > 0.0 or self.gust_std > 0.0

    def reset(self, env_ids):
        ids = slice(None) if env_ids is None else env_ids
        n = self.mean[ids].shape[0]
        spd = _u(n, *self.speed_range, self.device)
        ang = _u(n, 0.0, 2 * math.pi, self.device)
        self.mean[ids] = torch.stack([spd * torch.cos(ang), spd * torch.sin(ang), torch.zeros_like(spd)], dim=1)
        self.gust[ids] = 0.0

    def step(self) -> torch.Tensor:
        if self.gust_std > 0.0:
            k = self.dt / self.gust_tau_s
            self.gust += -self.gust * k + self.gust_std * math.sqrt(2.0 * k) * torch.randn_like(self.gust)
        return self.mean + self.gust


# --------------------------------------------------------------------------------------
# Wrench de los rotores
# --------------------------------------------------------------------------------------
def rotor_wrench(thrust_z: torch.Tensor, rotor_xy: torch.Tensor):
    """Fuerza y momento NETOS de los empujes de rotor respecto al CoM (origen del cuerpo).

    thrust_z : (N, 4) empuje de cada rotor a lo largo de +z del cuerpo [N]
    rotor_xy : (N, 4, 2) o (4, 2) posición (x, y) de cada rotor respecto al CoM [m]
    Devuelve (force (N,3), torque (N,3)). Con F=(0,0,Fz):  r x F = (y*Fz, -x*Fz, 0)  (la cota z del rotor no interviene).

    Se aplica TODO sobre el link base. Antes cada empuje se aplicaba al link del rotor en (x,y,z) expresado en el marco
    LOCAL de ese link, que ya está desplazado a la posición del rotor -> el brazo de palanca podía quedar duplicado.
    Aquí los brazos son exactamente los del modelo (los mismos que usa la matriz A del hover).
    """
    x, y = rotor_xy[..., 0], rotor_xy[..., 1]
    force = torch.zeros(thrust_z.shape[0], 3, device=thrust_z.device, dtype=thrust_z.dtype)
    torque = torch.zeros_like(force)
    force[:, 2] = thrust_z.sum(dim=1)
    torque[:, 0] = (y * thrust_z).sum(dim=1)
    torque[:, 1] = -(x * thrust_z).sum(dim=1)
    return force, torque


# --------------------------------------------------------------------------------------
# Referencia (punto guia)
# --------------------------------------------------------------------------------------
class ReferenceGenerator:
    """Genera pos/vel del punto guia en el marco local del entorno.

    Cambios respecto a env_17
    -------------------------
    1. Valla XY: el rebote solo se aplica si el objetivo SALE (signo pos*vel>0). Antes se invertia la velocidad en
       cada paso mientras |pos|>limite -> el objetivo quedaba "pegado" y vibrando en la valla.
    2. Aceleracion acotada (`a_max`): la velocidad sigue a una velocidad meta con limite de aceleracion, de modo que
       la referencia es dinamicamente factible (antes: inversiones instantaneas de 6 m/s).
    3. Arranque suave: la velocidad inicial del punto guia es la del dron, no la meta (antes: transitorio inevitable
       de 4-5 m de error en los primeros segundos).
    4. Limitador de yaw-rate muerto eliminado (target_yaw_prev == target_yaw => clamp era identidad).
    5. mode in {"random","circle","lemniscate"}: las dos ultimas son familias analiticas para evaluacion
       fuera de distribucion (no se usan en entrenamiento).
    """

    def __init__(self, num_envs: int, dt: float, device, *, a_max=1.5, speed_range=(3.0, 6.0), vz_init=0.4,
                 vz_change=0.8, p_change_z=0.01, yaw_rate_max=0.25, fence_xy=45.0, fence_margin=3.0,
                 z_limits=(1.75, 20.0), vmax_xy=6.0, t_ramp=5.0, mode="random"):
        self.n, self.dt, self.device, self.mode = num_envs, dt, device, mode
        self.a_max, self.speed_range, self.vz_init, self.vz_change = a_max, speed_range, vz_init, vz_change
        self.p_change_z, self.yaw_rate_max = p_change_z, yaw_rate_max
        self.fence = fence_xy - fence_margin
        self.z_limits, self.vmax_xy, self.t_ramp = z_limits, vmax_xy, t_ramp
        z = lambda *s: torch.zeros(*s, device=device)
        self.pos, self.vel, self.vel_goal = z(num_envs, 3), z(num_envs, 3), z(num_envs, 3)
        self.yaw_rate = z(num_envs)
        # parametros de las familias analiticas
        self.t, self.amp, self.w, self.phi, self.amp_z = z(num_envs), z(num_envs), z(num_envs), z(num_envs), z(num_envs)
        self.pos0 = z(num_envs, 3)

    # ------------------------------------------------------------------
    def reset(self, env_ids, pos0: torch.Tensor, vel0: torch.Tensor):
        ids = slice(None) if env_ids is None else env_ids
        n = self.pos[ids].shape[0]
        self.pos[ids] = pos0
        self.pos0[ids] = pos0
        self.t[ids] = 0.0
        if self.mode == "random":
            speed = _u(n, *self.speed_range, self.device)
            ang = _u(n, 0.0, 2 * math.pi, self.device)
            self.vel_goal[ids] = torch.stack(
                [speed * torch.cos(ang), speed * torch.sin(ang), _u(n, -self.vz_init, self.vz_init, self.device)], dim=1)
            self.vel[ids] = vel0                       # arranque suave
            self.yaw_rate[ids] = _u(n, -self.yaw_rate_max, self.yaw_rate_max, self.device)
        else:
            # circulo : |v| = amp*w          , |a| = amp*w^2           -> w <= a_max/vmax
            # lemnisc.: |v|max = sqrt(2)*amp*w, |a|max ~ 2.1*amp*w^2   -> w mas baja (x0.5) y amp = v/(1.5 w)
            v = _u(n, *self.speed_range, self.device)
            k, wf = (1.0, 1.0) if self.mode == "circle" else (1.5, 0.5)
            w = (self.a_max / self.vmax_xy) * wf * _u(n, 0.7, 1.0, self.device)
            self.w[ids], self.amp[ids] = w, v / (k * w)
            self.phi[ids] = _u(n, 0.0, 2 * math.pi, self.device)
            self.amp_z[ids] = _u(n, 1.0, 3.0, self.device)
            self.vel[ids] = vel0
            self.yaw_rate[ids] = w

    # ------------------------------------------------------------------
    def step(self):
        if self.mode == "random":
            self._step_random()
        else:
            self._step_analytic()

    def _step_random(self):
        dt = self.dt
        # 1) curvatura (rotacion de velocidad y meta alrededor de z)
        c, s = torch.cos(self.yaw_rate * dt), torch.sin(self.yaw_rate * dt)
        for v in (self.vel, self.vel_goal):
            vx, vy = v[:, 0].clone(), v[:, 1].clone()
            v[:, 0], v[:, 1] = vx * c - vy * s, vx * s + vy * c
        # 2) cambios aleatorios de velocidad vertical (sobre la META)
        ch = torch.rand(self.n, device=self.device) < self.p_change_z
        self.vel_goal[ch, 2] = _u(int(ch.sum()), -self.vz_change, self.vz_change, self.device)
        # 3) valla: rebote solo si el objetivo se aleja (signo correcto)
        for ax in (0, 1):
            out = (self.pos[:, ax].abs() > self.fence) & (self.pos[:, ax] * self.vel_goal[:, ax] > 0)
            self.vel_goal[out, ax] *= -1.0
        zlo, zhi = self.z_limits
        out_z = ((self.pos[:, 2] > zhi) & (self.vel_goal[:, 2] > 0)) | ((self.pos[:, 2] < zlo) & (self.vel_goal[:, 2] < 0))
        self.vel_goal[out_z, 2] *= -1.0
        # 4) velocidad con aceleracion acotada hacia la meta
        dv = self.vel_goal - self.vel
        scale = torch.clamp(self.a_max * dt / (dv.norm(dim=1, keepdim=True) + 1e-9), max=1.0)
        self.vel += dv * scale
        # 5) tope de velocidad horizontal
        sp = self.vel[:, :2].norm(dim=1, keepdim=True)
        self.vel[:, :2] *= torch.clamp(self.vmax_xy / (sp + 1e-6), max=1.0)
        # 6) integrar
        self.pos += self.vel * dt

    def _step_analytic(self):
        """Velocidad angular con rampa lineal 0 -> w en `t_ramp` s (arranque suave, aceleracion acotada)."""
        self.t += self.dt
        T = self.t_ramp
        r = torch.clamp(self.t / T, max=1.0)                                  # factor de rampa
        s = torch.where(self.t < T, self.t ** 2 / (2 * T), self.t - T / 2)    # integral de r(t)
        p, p0, rate = self.phi + self.w * s, self.phi, self.w * r
        if self.mode == "circle":
            rel = lambda q: torch.stack([self.amp * torch.cos(q), self.amp * torch.sin(q)], 1)
            vxy = torch.stack([-self.amp * rate * torch.sin(p), self.amp * rate * torch.cos(p)], 1)
        elif self.mode == "lemniscate":                     # lemniscata de Gerono (figura en 8)
            rel = lambda q: torch.stack([self.amp * torch.sin(q), 0.5 * self.amp * torch.sin(2 * q)], 1)
            vxy = torch.stack([self.amp * rate * torch.cos(p), self.amp * rate * torch.cos(2 * p)], 1)
        else:
            raise ValueError(f"mode desconocido: {self.mode}")
        dxy = rel(p) - rel(p0)
        dz = self.amp_z * (torch.sin(0.5 * p) - torch.sin(0.5 * p0))
        vz = 0.5 * self.amp_z * rate * torch.cos(0.5 * p)
        self.pos[:, :2] = self.pos0[:, :2] + dxy
        self.pos[:, 2] = self.pos0[:, 2] + dz
        self.vel[:, :2], self.vel[:, 2] = vxy, vz
        self.vel_goal.copy_(self.vel)
