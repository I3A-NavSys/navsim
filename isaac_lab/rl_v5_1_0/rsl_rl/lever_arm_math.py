"""Parte pura (numpy) del test de brazo de palanca. Sin Isaac: se puede probar en cualquier sitio.

Idea: se aplica un empuje conocido en UN rotor desde un hover limpio (sin inclinación, sin velocidad, masa nominal, tau=0,
sin viento), se mide la aceleración angular de la base y se compara con la que predice el modelo (brazos = posiciones del
USD, los de la matriz A del hover):
        tau_x = sum(y_i * dF_i)      tau_y = -sum(x_i * dF_i)      alpha = tau / I
    ratio = alpha_medida / alpha_modelo :  ~1  -> brazo del modelo (1x)   |   ~2 -> brazo duplicado   |   otro -> investigar
El eje z (reacción de par de los rotores, que se aplica como par sobre la base) NO depende del brazo: sirve de control del test.
"""
from __future__ import annotations
import numpy as np

KFT = np.array([4.6544, 4.6544, 0.9309, 0.9309])           # N / (rad/s)^2   (NW, NE, SW, SE)  -- los de env.py
KMDR = np.array([5.9683, 5.9683, 1.4921, 1.4921])          # N·m / (rad/s)^2 (valor absoluto)
POS = np.array([[0.5, 1.95], [0.5, -1.95], [-2.5, 1.55], [-2.5, -1.55]])   # (x, y) de cada rotor respecto al CoM
THRUST_SCALE = 0.5                                          # omega = omega_hover * (1 + 0.5 * a)

PATTERNS = {                                                # acción de cada rotor [NW, NE, SW, SE]
    "ROLL puro   [+.2,-.2,0,0]": [0.2, -0.2, 0.0, 0.0],        # solo balanceo (par izquierda-derecha de los delanteros)
    "PITCH puro  [0,0,+.5,+.5]": [0.0, 0.0, 0.5, 0.5],         # solo cabeceo (par de los traseros, el balanceo se anula)
    "MIXTO NW    [+.2,0,0,0]":   [0.2, 0.0, 0.0, 0.0],         # un rotor: balanceo + cabeceo + guiñada
    "ROLL puro (repetido)":      [0.2, -0.2, 0.0, 0.0],        # mismo patrón otra vez: debe dar lo mismo (independiente del orden)
}


def hover_u(mass_kg: float, g: float = 9.81) -> np.ndarray:
    """omega^2 de hover de cada rotor (misma matriz A que usa env.py)."""
    x, y = POS[:, 0], POS[:, 1]
    A = np.stack([KFT, y * KFT, -x * KFT, np.array([5.9683, -5.9683, 1.4921, -1.4921])])
    return np.linalg.solve(A, [mass_kg * g, 0.0, 0.0, 0.0])


def predict_alphas(action, inertia_diag, mass_kg, arm_scale: float = 1.0) -> np.ndarray:
    """Aceleración angular predicha [ax, ay, az] (rad/s^2) en ejes cuerpo. arm_scale=2 simula el brazo duplicado."""
    u_h = hover_u(mass_kg)
    u = u_h * (1.0 + THRUST_SCALE * np.asarray(action, float)) ** 2
    dF = KFT * (u - u_h)
    x, y = POS[:, 0], POS[:, 1]
    tau_x = arm_scale * float((y * dF).sum())
    tau_y = -arm_scale * float((x * dF).sum())
    tau_z = float(-KMDR[0] * u[0] + KMDR[1] * u[1] + KMDR[2] * u[2] - KMDR[3] * u[3])   # = MDR_z[1]-MDR_z[0]-MDR_z[3]+MDR_z[2]
    Ixx, Iyy, Izz = inertia_diag
    return np.array([tau_x / Ixx, tau_y / Iyy, tau_z / Izz])


def fit_alpha(omega_rec: np.ndarray, dt: float) -> np.ndarray:
    """Pendiente de omega(t) por eje, con t = dt, 2dt, ... (ajuste lineal con ordenada libre). omega_rec: (n, 3)."""
    t = dt * (np.arange(len(omega_rec)) + 1)
    return np.array([np.polyfit(t, omega_rec[:, i], 1)[0] for i in range(3)])


def classify(ratio: float, tol: float = 0.15) -> str:
    if abs(ratio - 1.0) <= tol: return "1x"
    if abs(ratio - 2.0) <= 2 * tol: return "2x"
    return "otro"


def verdict(measured: dict, predicted: dict, drift: np.ndarray | None = None, min_alpha: float = 0.05, tol: float = 0.15,
            leak_tol: float = 0.15, drift_tol: float = 0.10):
    """measured/predicted: {patrón: [ax, ay, az]} (measured ya sin la deriva de hover). drift: alpha medida con a=0 [ax, ay, az].
    Devuelve (veredicto, filas)."""
    rows, roll_pitch, yaw_ok, leaks = [], [], True, []
    for name in predicted:
        main = max(abs(predicted[name][0]), abs(predicted[name][1]), 1e-9)
        for ax, lab in enumerate(("roll", "pitch", "yaw")):
            p, m = predicted[name][ax], measured[name][ax]
            if abs(p) >= min_alpha:
                r = m / p; c = classify(r, tol)
                rows.append((name, lab, p, m, r, c))
                if lab == "yaw": yaw_ok &= (c == "1x")
                else: roll_pitch.append(c)
            elif lab != "yaw":                                   # el modelo dice ~0: lo medido es "fuga" entre ejes
                fuga = abs(m) / max(max(abs(measured[name][0]), abs(measured[name][1])), 1e-9) if abs(m) > 0 else 0.0
                c = "fuga" if (abs(m) > min_alpha and fuga > leak_tol) else "ok"
                rows.append((name, lab, p, m, float("nan"), c)); leaks.append(c)
    # repetibilidad: los patrones repetidos deben coincidir
    names = [n for n in measured if "repetido" in n]
    rep_ok = True
    for n in names:
        base = n.replace(" (repetido)", "")
        ref = next((k for k in measured if k.startswith(base)), None)
        if ref is not None:
            rep_ok &= bool(np.allclose(measured[n][:2], measured[ref][:2], rtol=0.15, atol=0.05))
    drift_txt = ""
    if drift is not None and np.max(np.abs(drift[:2])) > drift_tol:
        drift_txt = f" · AVISO: hover no equilibrado (deriva a=0: {np.round(drift, 3)} rad/s²)"
    if not yaw_ok:
        return "TEST INVÁLIDO: el eje z (que no depende del brazo) no cuadra con el modelo" + drift_txt, rows
    if not rep_ok:
        return "TEST NO REPETIBLE: el mismo patrón da resultados distintos (depende del estado/orden)" + drift_txt, rows
    bad_leak = any(c == "fuga" for c in leaks)
    if roll_pitch and all(c == "1x" for c in roll_pitch) and not bad_leak:
        return "BRAZO 1x: los momentos coinciden con el modelo (sin fugas entre ejes)" + drift_txt, rows
    if roll_pitch and all(c == "2x" for c in roll_pitch) and not bad_leak:
        return "BRAZO 2x: duplicación CONFIRMADA" + drift_txt, rows
    return "OTRO: ni 1x ni 2x de forma consistente (hay acoplamiento entre ejes o factores distintos)" + drift_txt, rows
