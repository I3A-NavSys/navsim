import ast
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence
import numpy as np

try:
    import yaml as _yaml
except ImportError:
    _yaml = None


FLEET_DIR = Path(__file__).resolve().parents[3] / "fleet"


def _load_yaml_file(yaml_path: Path) -> Dict[str, Any]:
    """
    Carga un fichero `dynamics.yaml`. Usa PyYAML si está instalado o un parser
    nativo ligero para el subconjunto declarativo de `dynamics.yaml` sin dependencias.
    """
    text = yaml_path.read_text(encoding="utf-8")
    if _yaml is not None:
        return _yaml.safe_load(text)

    root: Dict[str, Any] = {}
    current_section: Optional[str] = None
    current_list_key: Optional[str] = None

    for raw_line in text.splitlines():
        # Eliminar comentarios manteniendo contenido previo
        line = raw_line.split("#", 1)[0].rstrip()
        if not line.strip():
            continue

        indent = len(line) - len(line.lstrip(" "))
        stripped = line.strip()

        if indent == 0 and stripped.endswith(":"):
            key = stripped[:-1].strip()
            current_section = key
            current_list_key = None
            root[key] = {}
        elif indent == 0 and ":" in stripped:
            k, v = stripped.split(":", 1)
            k = k.strip()
            v = v.strip()
            current_section = None
            if v:
                root[k] = ast.literal_eval(v)
            else:
                root[k] = []
                current_list_key = k
        elif indent > 0 and stripped.startswith("- "):
            item_val = ast.literal_eval(stripped[2:].strip())
            if current_section is not None and current_list_key is not None:
                root[current_section][current_list_key].append(item_val)
            elif current_list_key is not None:
                root[current_list_key].append(item_val)
        elif indent > 0 and ":" in stripped and current_section is not None:
            k, v = stripped.split(":", 1)
            k = k.strip()
            v = v.strip()
            if v:
                root[current_section][k] = ast.literal_eval(v)
                current_list_key = None
            else:
                root[current_section][k] = []
                current_list_key = k

    return root


@dataclass(frozen=True)
class DroneModelConfig:
    """
    Ficha técnica completa (física, aerodinámica y matrices de control lineal)
    de un modelo de dron cargada desde `fleet/<modelo>/dynamics.yaml`.
    """
    model_name: str
    folder_path: Path
    usd_file: str
    aliases: List[str] = field(default_factory=list)

    # Propiedades físicas e inerciales
    gravity: float = 9.81
    mass: float = 2000.0
    inertia: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    bodies_positions: np.ndarray = field(default_factory=lambda: np.zeros((5, 3), dtype=np.float64))

    # Parámetros aerodinámicos y de propulsión
    w_min: float = 0.0
    w_max: float = 62.8319
    w_hov: float = 41.8879
    kFT: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.float64))
    kMDR: np.ndarray = field(default_factory=lambda: np.zeros(4, dtype=np.float64))
    kFD: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    kMD: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))

    # Matrices de control lineal en espacio de estados: u = Hs - Kx @ x - Ky @ E
    t_to_solve: float = 2.0
    E_max: float = 150.0
    Hs: np.ndarray = field(default_factory=lambda: np.zeros((4, 1), dtype=np.float64))
    Kx: np.ndarray = field(default_factory=lambda: np.zeros((4, 8), dtype=np.float64))
    Ky: np.ndarray = field(default_factory=lambda: np.zeros((4, 4), dtype=np.float64))

    @property
    def usd_path(self) -> Path:
        return self.folder_path / self.usd_file

    @classmethod
    def from_yaml(cls, yaml_path: Path) -> "DroneModelConfig":
        data = _load_yaml_file(yaml_path)

        physics = data["physics"]
        aero = data["aerodynamics"]
        ctrl = data["control"]

        bodies_pos = np.asarray(physics["bodies_positions"], dtype=np.float64)
        if bodies_pos.shape != (5, 3):
            raise ValueError(
                f"[{yaml_path}] 'bodies_positions' debe tener dimensión (5, 3) "
                f"[CoM, NE, NW, SE, SW], recibido: {bodies_pos.shape}"
            )

        kx = np.asarray(ctrl["Kx"], dtype=np.float64)
        ky = np.asarray(ctrl["Ky"], dtype=np.float64)
        hs = np.asarray(ctrl["Hs"], dtype=np.float64).reshape(4, 1)

        if kx.shape != (4, 8):
            raise ValueError(f"[{yaml_path}] 'Kx' debe ser (4, 8), recibido: {kx.shape}")
        if ky.shape != (4, 4):
            raise ValueError(f"[{yaml_path}] 'Ky' debe ser (4, 4), recibido: {ky.shape}")

        return cls(
            model_name=str(data["model_name"]),
            folder_path=yaml_path.parent,
            usd_file=str(data.get("usd_file", "1_rb_model.usd")),
            aliases=[str(a) for a in data.get("aliases", [])],
            gravity=float(physics.get("gravity", 9.81)),
            mass=float(physics["mass"]),
            inertia=np.asarray(physics["inertia"], dtype=np.float64),
            bodies_positions=bodies_pos,
            w_min=float(aero["w_min"]),
            w_max=float(aero["w_max"]),
            w_hov=float(aero["w_hov"]),
            kFT=np.asarray(aero["kFT"], dtype=np.float64),
            kMDR=np.asarray(aero["kMDR"], dtype=np.float64),
            kFD=np.asarray(aero["kFD"], dtype=np.float64),
            kMD=np.asarray(aero["kMD"], dtype=np.float64),
            t_to_solve=float(ctrl.get("t_to_solve", 2.0)),
            E_max=float(ctrl["E_max"]),
            Hs=hs,
            Kx=kx,
            Ky=ky,
        )


@dataclass
class FleetDynamicsTensors:
    """
    Tensores NumPy pre-apilados para N drones (potencialmente heterogéneos).
    Permite ejecutar el control lineal y la dinámica en un único bloque matricial.
    """
    count: int
    model_names: List[str]

    # Física e inercia
    mass: np.ndarray                    # (N,)
    inertia: np.ndarray                 # (N, 3)
    bodies_positions: np.ndarray        # (N, 5, 3) [CoM, NE, NW, SE, SW]

    # Límites y constantes aerodinámicas
    w_min: np.ndarray                   # (N, 1, 1)
    w_max: np.ndarray                   # (N, 1, 1)
    w_hov: np.ndarray                   # (N,)
    thrust_coeffs: np.ndarray           # (N, 4)     -> +kFT
    drag_moment_coeffs: np.ndarray      # (N, 4)     -> kMDR (con signo de giro incluido)
    drag_force_coeffs: np.ndarray       # (N, 3)     -> -kFD
    friction_moment_coeffs: np.ndarray  # (N, 3)     -> -kMD

    # Matrices de control lineal por dron
    t_to_solve: np.ndarray              # (N, 1)
    E_max: np.ndarray                   # (N, 1, 1)
    Hs: np.ndarray                      # (N, 4, 1)
    Kx: np.ndarray                      # (N, 4, 8)
    Ky: np.ndarray                      # (N, 4, 4)


class FleetCatalog:
    """
    Catálogo dinámico de tipos de dron disponibles en `fleet/`.
    Descubre automáticamente cualquier carpeta que contenga un `dynamics.yaml`.
    """

    def __init__(self, fleet_dir: Path = FLEET_DIR, default_model: str = "UAM_aerotaxi"):
        self.fleet_dir = Path(fleet_dir)
        self.default_model = default_model
        self.models: Dict[str, DroneModelConfig] = {}
        self.alias_map: Dict[str, str] = {}
        self.reload()

    def reload(self) -> None:
        self.models.clear()
        self.alias_map.clear()

        for yaml_path in sorted(self.fleet_dir.glob("*/dynamics.yaml")):
            cfg = DroneModelConfig.from_yaml(yaml_path)
            self.models[cfg.model_name] = cfg

            # Registrar nombre exacto, nombre de carpeta y alias en minúsculas
            keys = {cfg.model_name, yaml_path.parent.name, *cfg.aliases}
            for k in keys:
                self.alias_map[k.strip().lower()] = cfg.model_name

    def get(self, name_or_alias: Optional[str]) -> DroneModelConfig:
        if not name_or_alias:
            return self.models[self.default_model]
        key = name_or_alias.strip().lower()
        if key in self.alias_map:
            return self.models[self.alias_map[key]]
        for alias_key, canonical_name in self.alias_map.items():
            if alias_key in key:
                return self.models[canonical_name]
        return self.models[self.default_model]

    def build_fleet_tensors(self, model_names: Sequence[str]) -> FleetDynamicsTensors:
        configs = [self.get(name) for name in model_names]
        n = len(configs)
        if n == 0:
            raise ValueError("No se puede construir FleetDynamicsTensors para 0 drones.")

        return FleetDynamicsTensors(
            count=n,
            model_names=[c.model_name for c in configs],
            mass=np.array([c.mass for c in configs], dtype=np.float64),
            inertia=np.stack([c.inertia for c in configs], axis=0),
            bodies_positions=np.stack([c.bodies_positions for c in configs], axis=0),
            w_min=np.array([c.w_min for c in configs], dtype=np.float64).reshape(n, 1, 1),
            w_max=np.array([c.w_max for c in configs], dtype=np.float64).reshape(n, 1, 1),
            w_hov=np.array([c.w_hov for c in configs], dtype=np.float64),
            thrust_coeffs=np.stack([c.kFT for c in configs], axis=0),
            drag_moment_coeffs=np.stack([c.kMDR for c in configs], axis=0),
            drag_force_coeffs=-np.stack([c.kFD for c in configs], axis=0),
            friction_moment_coeffs=-np.stack([c.kMD for c in configs], axis=0),
            t_to_solve=np.array([c.t_to_solve for c in configs], dtype=np.float64).reshape(n, 1),
            E_max=np.array([c.E_max for c in configs], dtype=np.float64).reshape(n, 1, 1),
            Hs=np.stack([c.Hs for c in configs], axis=0),
            Kx=np.stack([c.Kx for c in configs], axis=0),
            Ky=np.stack([c.Ky for c in configs], axis=0),
        )
