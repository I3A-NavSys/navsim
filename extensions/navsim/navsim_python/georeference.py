"""
Módulo de Georreferenciación WGS-84 y Proyecciones Locales ENU para NavSim.

Define el plano tangente local East-North-Up (ENU) anclado a un origen geodésico
(Latitud, Longitud, Altitud Elipsoidal WGS-84), permitiendo la transformación bidireccional
entre coordenadas cartesianas locales de Isaac Sim (x=Este, y=Norte, z=Arriba) y WGS-84.
"""

from typing import Optional, Tuple
import numpy as np

# Coordenadas por defecto: Campus Universitario de Albacete (UCLM - ESII / I3A)
DEFAULT_CAMPUS_ALBACETE_LAT = 38.981800
DEFAULT_CAMPUS_ALBACETE_LON = -1.857000
DEFAULT_CAMPUS_ALBACETE_ALT = 686.00


class WGS84Georeference:
    """
    Gestor de transformaciones geodésicas sobre el elipsoide WGS-84.
    Mapea el origen cartesiano (0, 0, 0) de Isaac Sim a un plano tangente local ENU:
      +X: Dirección Este (geodésica)
      +Y: Dirección Norte (geodésica)
      +Z: Dirección Arriba (normal al elipsoide WGS-84)
    """

    def __init__(
        self,
        lat0: float = DEFAULT_CAMPUS_ALBACETE_LAT,
        lon0: float = DEFAULT_CAMPUS_ALBACETE_LON,
        alt0: float = DEFAULT_CAMPUS_ALBACETE_ALT,
        source: str = "Predefinido (Campus Albacete)",
    ):
        # Constantes del elipsoide WGS-84
        self.a = 6378137.0  # Semieje mayor (m)
        self.f = 1.0 / 298.257223563  # Achatamiento
        self.b = self.a * (1.0 - self.f)  # Semieje menor (m)
        self.e2 = (self.a**2 - self.b**2) / (self.a**2)  # Primera excentricidad al cuadrado
        self.e_prime2 = (self.a**2 - self.b**2) / (self.b**2)  # Segunda excentricidad al cuadrado

        self.lat0: float = float(lat0)
        self.lon0: float = float(lon0)
        self.alt0: float = float(alt0)
        self.source: str = source

        self._update_matrices()

    def set_origin(self, lat0: float, lon0: float, alt0: float, source: Optional[str] = None) -> None:
        """Establece un nuevo punto de anclaje (0, 0, 0) para el mundo."""
        self.lat0 = float(lat0)
        self.lon0 = float(lon0)
        self.alt0 = float(alt0)
        if source is not None:
            self.source = source
        self._update_matrices()

    def _update_matrices(self) -> None:
        phi = np.radians(self.lat0)
        lam = np.radians(self.lon0)

        sin_phi = np.sin(phi)
        cos_phi = np.cos(phi)
        sin_lam = np.sin(lam)
        cos_lam = np.cos(lam)

        # Radio de curvatura en el primer vertical
        N0 = self.a / np.sqrt(1.0 - self.e2 * sin_phi**2)

        # Origen ECEF del punto de anclaje
        x0 = (N0 + self.alt0) * cos_phi * cos_lam
        y0 = (N0 + self.alt0) * cos_phi * sin_lam
        z0 = (N0 * (1.0 - self.e2) + self.alt0) * sin_phi
        self.ecef0 = np.array([x0, y0, z0], dtype=np.float64)

        # Matriz de rotación: ENU -> ECEF
        self.R_enu_to_ecef = np.array([
            [-sin_lam, -sin_phi * cos_lam, cos_phi * cos_lam],
            [ cos_lam, -sin_phi * sin_lam, cos_phi * sin_lam],
            [ 0.0,      cos_phi,           sin_phi          ],
        ], dtype=np.float64)

        # Matriz inversa: ECEF -> ENU
        self.R_ecef_to_enu = self.R_enu_to_ecef.T

    def enu_to_geodetic(self, enu: np.ndarray) -> np.ndarray:
        """
        Convierte coordenadas locales ENU (x=Este, y=Norte, z=Arriba) en metros
        a geodésicas WGS-84 (Latitud [°], Longitud [°], Altitud [m]).

        Soporta vectores (3,) o matrices vectorizadas (N, 3).
        """
        is_1d = (enu.ndim == 1)
        enu_2d = np.atleast_2d(enu)

        # 1. Transformar ENU a ECEF
        ecef = self.ecef0 + enu_2d @ self.R_enu_to_ecef.T
        X = ecef[:, 0]
        Y = ecef[:, 1]
        Z = ecef[:, 2]

        # 2. Transformar ECEF a Geodésicas mediante el algoritmo analítico de Bowring
        p = np.sqrt(X**2 + Y**2)
        theta = np.arctan2(Z * self.a, p * self.b)
        
        sin_th = np.sin(theta)
        cos_th = np.cos(theta)
        
        lat = np.arctan2(
            Z + self.e_prime2 * self.b * (sin_th**3),
            p - self.e2 * self.a * (cos_th**3),
        )
        lon = np.arctan2(Y, X)

        sin_lat = np.sin(lat)
        cos_lat = np.cos(lat)
        N = self.a / np.sqrt(1.0 - self.e2 * sin_lat**2)
        alt = p / cos_lat - N

        res = np.column_stack([np.degrees(lat), np.degrees(lon), alt])
        return res[0] if is_1d else res

    def geodetic_to_enu(self, lat: float, lon: float, alt: float) -> np.ndarray:
        """
        Convierte una coordenada geodésica WGS-84 a coordenadas locales ENU (x, y, z) en metros.
        """
        phi = np.radians(lat)
        lam = np.radians(lon)

        N = self.a / np.sqrt(1.0 - self.e2 * np.sin(phi)**2)
        X = (N + alt) * np.cos(phi) * np.cos(lam)
        Y = (N + alt) * np.cos(phi) * np.sin(lam)
        Z = (N * (1.0 - self.e2) + alt) * np.sin(phi)

        ecef = np.array([X, Y, Z], dtype=np.float64)
        delta_ecef = ecef - self.ecef0
        return self.R_ecef_to_enu @ delta_ecef

    @staticmethod
    def detect_from_stage(stage) -> Tuple[float, float, float, str, bool]:
        """
        Inspecciona el escenario USD para descubrir si ya tiene definidas
        coordenadas geodésicas de anclaje (estándar NVIDIA WGS84, Cesium o NavSim).

        Devuelve (lat, lon, alt, source_label, is_detected).
        """
        if not stage:
            return (
                DEFAULT_CAMPUS_ALBACETE_LAT,
                DEFAULT_CAMPUS_ALBACETE_LON,
                DEFAULT_CAMPUS_ALBACETE_ALT,
                "Predefinido (Campus Albacete)",
                False,
            )

        # 1. Comprobar /World (estándar NVIDIA OpenUSD Geospatial)
        world = stage.GetPrimAtPath("/World")
        if world and world.IsValid():
            attr_ref = world.GetAttribute("omni:geospatial:wgs84:reference:referencePosition")
            if attr_ref and attr_ref.IsValid() and attr_ref.Get() is not None:
                val = attr_ref.Get()
                return float(val[0]), float(val[1]), float(val[2]), "USD: /World (NVIDIA WGS84)", True

            attr_lat = world.GetAttribute("NavSim:georeference:latitude")
            attr_lon = world.GetAttribute("NavSim:georeference:longitude")
            attr_alt = world.GetAttribute("NavSim:georeference:altitude")
            if attr_lat and attr_lat.IsValid() and attr_lat.Get() is not None:
                lat = float(attr_lat.Get())
                lon = float(attr_lon.Get()) if attr_lon and attr_lon.IsValid() else 0.0
                alt = float(attr_alt.Get()) if attr_alt and attr_alt.IsValid() else 0.0
                return lat, lon, alt, "USD: /World (NavSim)", True

        # 2. Comprobar si hay algún Prim con CesiumGeoreference o schema WGS84 en el escenario
        for prim in stage.Traverse():
            attr_ref = prim.GetAttribute("omni:geospatial:wgs84:reference:referencePosition")
            if attr_ref and attr_ref.IsValid() and attr_ref.Get() is not None:
                val = attr_ref.Get()
                return float(val[0]), float(val[1]), float(val[2]), f"USD: {prim.GetName()} (NVIDIA WGS84)", True

            if prim.GetTypeName() == "CesiumGeoreference":
                lat_attr = prim.GetAttribute("cesium:georeference:latitude")
                lon_attr = prim.GetAttribute("cesium:georeference:longitude")
                h_attr = prim.GetAttribute("cesium:georeference:height")
                if lat_attr and lat_attr.IsValid() and lat_attr.Get() is not None:
                    lat = float(lat_attr.Get())
                    lon = float(lon_attr.Get()) if lon_attr else 0.0
                    alt = float(h_attr.Get()) if h_attr else 0.0
                    return lat, lon, alt, "USD: CesiumGeoreference", True

        # 3. No encontrado -> Valor por defecto Campus de Albacete
        return (
            DEFAULT_CAMPUS_ALBACETE_LAT,
            DEFAULT_CAMPUS_ALBACETE_LON,
            DEFAULT_CAMPUS_ALBACETE_ALT,
            "Predefinido (Campus Albacete)",
            False,
        )
