"""
Módulo de Cronorreferenciación y Gestión del Tiempo Real/Simulado para NavSim.

Define el anclaje temporal de referencia (T0), permitiendo la evolución coherente
del tiempo real en función del tiempo de simulación acumulado (t_sim):
    T_real(t) = T0 + t_sim

Soporta autodescubrimiento desde metadatos USD, sincronización con el reloj del sistema
o configuración manual por parte del operador.
"""

from datetime import datetime
import logging
from typing import Optional, Tuple


class NavSimTimeReference:
    """
    Gestor de referencia temporal para NavSim.
    Mapea el instante inicial (sim_time = 0.0) a una fecha y hora (T0).
    """

    def __init__(
        self,
        epoch_unix: Optional[float] = None,
        source: str = "Reloj del Sistema",
    ):
        self.logger = logging.getLogger("NavSimTimeReference")
        if epoch_unix is None:
            now = datetime.now()
            self.epoch_unix: float = float(now.timestamp())
            self.source: str = "Reloj del Sistema"
        else:
            self.epoch_unix = float(epoch_unix)
            self.source = source
        self.end_epoch_unix: float = self.epoch_unix + 3600.0

    @staticmethod
    def parse_date_str(date_str: str) -> Optional[str]:
        """
        Valida e interpreta una cadena de fecha, admitiendo formatos AAAA-MM-DD, AAAA/MM/DD,
        DD-MM-AAAA y DD/MM/AAAA. Retorna la fecha normalizada en formato 'AAAA-MM-DD', o None si es inválida.
        """
        if not date_str or not isinstance(date_str, str):
            return None
        clean = date_str.strip().replace("/", "-")
        for fmt in ("%Y-%m-%d", "%d-%m-%Y"):
            try:
                dt = datetime.strptime(clean, fmt)
                return dt.strftime("%Y-%m-%d")
            except ValueError:
                pass
        return None

    @staticmethod
    def parse_time_str(time_str: str) -> Optional[str]:
        """
        Valida e interpreta una cadena de hora, admitiendo formatos HH:MM:SS o HH:MM
        (separados por ':' o '.'). Retorna la hora normalizada en formato 'HH:MM:SS', o None si es inválida.
        """
        if not time_str or not isinstance(time_str, str):
            return None
        clean = time_str.strip().replace(".", ":")
        parts = clean.split(":")
        if len(parts) == 2:
            clean = f"{clean}:00"
        elif len(parts) != 3:
            return None
        try:
            dt = datetime.strptime(clean, "%H:%M:%S")
            return dt.strftime("%H:%M:%S")
        except ValueError:
            return None

    def set_datetime(
        self,
        date_str: str,
        time_str: str,
        source: str = "Manual (Interfaz)",
    ) -> bool:
        """
        Configura la fecha (AAAA-MM-DD) y hora (HH:MM:SS) de referencia.
        Retorna True si el formato fue válido y se aplicó con éxito.
        """
        valid_date = self.parse_date_str(date_str)
        valid_time = self.parse_time_str(time_str)

        if not valid_date or not valid_time:
            self.logger.warning(
                f"Formato de fecha/hora inválido. Fecha: '{date_str}' (válida: {bool(valid_date)}), "
                f"Hora: '{time_str}' (válida: {bool(valid_time)})"
            )
            return False

        iso_str = f"{valid_date} {valid_time}"
        try:
            dt = datetime.strptime(iso_str, "%Y-%m-%d %H:%M:%S")
            old_duration = max(3600.0, getattr(self, "end_epoch_unix", 0.0) - getattr(self, "epoch_unix", 0.0))
            self.epoch_unix = float(dt.timestamp())
            self.end_epoch_unix = self.epoch_unix + old_duration
            self.source = source
            return True
        except ValueError as e:
            self.logger.warning(f"Error interpretando fecha/hora '{iso_str}': {e}")
            return False

    def set_from_datetime(self, dt: datetime, source: str = "Manual (Interfaz)") -> None:
        """Configura el anclaje temporal a partir de un objeto datetime."""
        self.epoch_unix = float(dt.timestamp())
        self.source = source

    def reset_to_system_clock(self) -> None:
        """Restablece el anclaje temporal a la fecha y hora actual del reloj del sistema."""
        now = datetime.now()
        self.epoch_unix = float(now.timestamp())
        self.end_epoch_unix = self.epoch_unix + 3600.0
        self.source = "Reloj del Sistema"

    def set_end_datetime(self, date_str: str, time_str: str) -> bool:
        """Configura el instante final programado para la simulación a partir de fecha (AAAA-MM-DD) y hora (HH:MM:SS)."""
        valid_date = self.parse_date_str(date_str)
        valid_time = self.parse_time_str(time_str)
        if not valid_date or not valid_time:
            return False
        iso_str = f"{valid_date} {valid_time}"
        try:
            dt = datetime.strptime(iso_str, "%Y-%m-%d %H:%M:%S")
            self.end_epoch_unix = float(dt.timestamp())
            return True
        except ValueError:
            return False

    def set_end_plus_hours(self, hours: float = 1.0) -> None:
        """
        Suma de forma acumulativa las horas indicadas al instante final programado.
        Si el instante final es anterior o igual al instante inicial, parte del inicial sumando las horas.
        Si ya es posterior al inicial, incrementa acumulativamente el instante final.
        """
        step = max(0.0, float(hours)) * 3600.0
        if getattr(self, "end_epoch_unix", 0.0) <= self.epoch_unix:
            self.end_epoch_unix = self.epoch_unix + step
        else:
            self.end_epoch_unix += step

    def get_end_datetime(self) -> datetime:
        """Retorna el objeto datetime para el instante final programado."""
        return datetime.fromtimestamp(self.end_epoch_unix)

    def get_end_date_str(self) -> str:
        """Retorna la fecha en formato 'AAAA-MM-DD' para el instante final programado."""
        return self.get_end_datetime().strftime("%Y-%m-%d")

    def get_end_time_str(self) -> str:
        """Retorna la hora en formato 'HH:MM:SS' para el instante final programado."""
        return self.get_end_datetime().strftime("%H:%M:%S")

    def get_target_duration_seconds(self) -> float:
        """Retorna la duración total en segundos desde el instante inicial hasta el final."""
        return max(0.0, self.end_epoch_unix - self.epoch_unix)

    def has_reached_end(self, sim_time: float) -> bool:
        """Determina si el tiempo transcurrido de simulación ha alcanzado o superado el instante final programado."""
        if self.end_epoch_unix <= self.epoch_unix:
            return False
        return (self.epoch_unix + float(sim_time)) >= self.end_epoch_unix

    def get_datetime(self, sim_time: float = 0.0) -> datetime:
        """Retorna el objeto datetime para el instante actual de simulación (T0 + sim_time)."""
        current_ts = self.epoch_unix + float(sim_time)
        return datetime.fromtimestamp(current_ts)

    def get_date_str(self, sim_time: float = 0.0) -> str:
        """Retorna la fecha en formato 'AAAA-MM-DD' para el instante actual de simulación."""
        dt = self.get_datetime(sim_time)
        return dt.strftime("%Y-%m-%d")

    def get_time_str(self, sim_time: float = 0.0) -> str:
        """Retorna la hora en formato 'HH:MM:SS' para el instante actual de simulación."""
        dt = self.get_datetime(sim_time)
        return dt.strftime("%H:%M:%S")

    def get_unix_timestamp(self, sim_time: float = 0.0) -> float:
        """Retorna el timestamp Unix en segundos con coma flotante (T0 + sim_time)."""
        return self.epoch_unix + float(sim_time)

    def get_odid_timestamp(self, sim_time: float = 0.0) -> float:
        """
        Retorna los segundos transcurridos desde el inicio de la hora UTC actual,
        según la especificación de Remote ID ASTM F3411 / OpenDroneID (rango 0.0 .. 3599.9 s).
        """
        current_unix = self.epoch_unix + float(sim_time)
        return float(current_unix % 3600.0)

    @classmethod
    def detect_from_stage(cls, stage) -> Tuple["NavSimTimeReference", bool]:
        """
        Inspecciona el escenario USD para descubrir si ya tiene definida una fecha/hora
        de referencia (metadatos en /World o en customLayerData del layer raíz).

        Retorna (instancia_NavSimTimeReference, fue_detectado).
        """
        if not stage:
            return cls(source="Reloj del Sistema"), False

        # 1. Comprobar /World (atributos personalizados NavSim)
        world = stage.GetPrimAtPath("/World")
        if world and world.IsValid():
            attr_iso = world.GetAttribute("NavSim:time:datetime")
            if attr_iso and attr_iso.IsValid() and attr_iso.Get() is not None:
                val_str = str(attr_iso.Get()).strip()
                ref = cls()
                val_str = val_str.replace("Z", "").strip()
                parts = val_str.split("T") if "T" in val_str else val_str.split(" ")
                if len(parts) == 2 and ref.set_datetime(parts[0], parts[1], source="USD: /World (NavSim)"):
                    return ref, True

            attr_epoch = world.GetAttribute("NavSim:time:epoch")
            if attr_epoch and attr_epoch.IsValid() and attr_epoch.Get() is not None:
                epoch_val = float(attr_epoch.Get())
                return cls(epoch_unix=epoch_val, source="USD: /World (NavSim)"), True

        # 2. Comprobar customLayerData en el Layer raíz del stage USD
        try:
            root_layer = stage.GetRootLayer()
            if root_layer and hasattr(root_layer, "customLayerData"):
                data = root_layer.customLayerData
                if isinstance(data, dict):
                    # Buscar claves reconocidas
                    val_iso = data.get("navsim:time_datetime") or data.get("navsim:datetime") or data.get("time_datetime")
                    if val_iso:
                        val_str = str(val_iso).strip()
                        ref = cls()
                        val_str = val_str.replace("Z", "").strip()
                        parts = val_str.split("T") if "T" in val_str else val_str.split(" ")
                        if len(parts) == 2 and ref.set_datetime(parts[0], parts[1], source="USD: Metadatos"):
                            return ref, True

                    val_epoch = data.get("navsim:time_epoch") or data.get("navsim:epoch") or data.get("time_epoch")
                    if val_epoch is not None:
                        return cls(epoch_unix=float(val_epoch), source="USD: Metadatos"), True
        except Exception:
            pass

        # 3. No encontrado en USD -> Reloj del sistema actual
        return cls(source="Reloj del Sistema"), False
