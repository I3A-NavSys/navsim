"""
Módulo de Identificación Estándar y Validación de Flota ASTM F3411 / EUROCAE ED-282 para NavSim.

Define los tipos y atributos oficiales de identificación Open Remote ID (Direct & Network Remote ID)
según la normativa internacional ASTM F3411-22a y el estándar europeo EUROCAE ED-282 / Delegated Reg. (EU) 2019/945:
  1. Identificación básica de la aeronave (UAS ID, Número de serie ANSI/CTA-2063-A).
  2. Categoría y tipo de aeronave (UA Type: Multirrotor, Ala fija, VTOL híbrido...).
  3. Tipo de identificación (Número de serie, registro CAA, UUID UTM, ID de sesión).
  4. Identificación oficial de registro de operador (CAA / EASA Operator Registration).
  5. Descripción de misión o texto libre (Self ID).

Incluye validadores en tiempo real para:
  - Detección inmediata de conflictos por duplicidad de identificadores en el espacio aéreo.
  - Validación de campos obligatorios faltantes o erróneos en cada Prim de UAV del escenario USD.
  - Reconocimiento dinámico de aeronaves por atributos NavSim, sin restricción sobre el nombre del Prim.
"""

from dataclasses import dataclass, field
from enum import IntEnum
import re
from typing import Any, Dict, List, Optional, Set, Tuple


# ==============================================================================
# Enumerados Oficiales ASTM F3411 / EUROCAE ED-282
# ==============================================================================
class ASTMIdType(IntEnum):
    """Tipos de Identificador de Aeronave (ASTM F3411 / MAV_ODID_ID_TYPE)."""
    NONE = 0
    SERIAL_NUMBER = 1          # Número de serie de fabricante (ANSI/CTA-2063-A)
    CAA_REGISTRATION = 2       # Registro oficial emitido por la Autoridad de Aviación Civil (AESA, FAA...)
    UTM_ASSIGNED_UUID = 3      # UUID asignado dinámicamente por proveedor de servicios UTM/U-Space
    SPECIFIC_SESSION_ID = 4    # Identificador temporal de sesión específica


class ASTMUaType(IntEnum):
    """Categoría física de la aeronave (ASTM F3411 / MAV_ODID_UA_TYPE)."""
    UNDECLARED = 0
    AEROPLANE = 1              # Ala fija
    HELICOPTER_MULTIROTOR = 2  # Multirrotor / Helicóptero
    GYROPLANE = 3              # Autogiro
    HYBRID_LIFT_VTOL = 4       # VTOL de sustentación híbrida
    ORNITHOPTER = 5            # Ornitóptero
    GLIDER = 6                 # Planeador
    KITE = 7                   # Cometa
    FREE_BALLOON = 8           # Globo libre
    CAPTIVE_BALLOON = 9        # Globo cautivo
    AIRSHIP = 10               # Dirigible
    FREE_FALL_PARACHUTE = 11   # Paracaídas
    ROCKET = 12                # Cohete
    TETHERED_POWERED = 13      # Aeronave anclada alimentada
    GROUND_OBSTACLE = 14       # Obstáculo terrestre


class ASTMOperatorIdType(IntEnum):
    """Tipo de registro del operador (ASTM F3411 / MAV_ODID_OPERATOR_ID_TYPE)."""
    CAA_OPERATOR_ID = 0        # Registro oficial de operador CAA/EASA/FAA


ASTM_ID_TYPE_NAMES: Dict[int, str] = {
    int(ASTMIdType.NONE): "Ninguno",
    int(ASTMIdType.SERIAL_NUMBER): "Nº Serie (ANSI/CTA-2063-A)",
    int(ASTMIdType.CAA_REGISTRATION): "Registro Oficial CAA/AESA",
    int(ASTMIdType.UTM_ASSIGNED_UUID): "UUID UTM/U-Space",
    int(ASTMIdType.SPECIFIC_SESSION_ID): "ID de Sesión Específica",
}

ASTM_UA_TYPE_NAMES: Dict[int, str] = {
    int(ASTMUaType.UNDECLARED): "No declarado",
    int(ASTMUaType.AEROPLANE): "Aeroplano (Ala fija)",
    int(ASTMUaType.HELICOPTER_MULTIROTOR): "Helicóptero / Multirrotor",
    int(ASTMUaType.GYROPLANE): "Autogiro / Giroplano",
    int(ASTMUaType.HYBRID_LIFT_VTOL): "VTOL Híbrido",
    int(ASTMUaType.ORNITHOPTER): "Ornitóptero",
    int(ASTMUaType.GLIDER): "Planeador",
    int(ASTMUaType.AIRSHIP): "Dirigible",
}

_STR_TO_ID_TYPE: Dict[str, ASTMIdType] = {
    "SERIAL_NUMBER": ASTMIdType.SERIAL_NUMBER,
    "SERIAL": ASTMIdType.SERIAL_NUMBER,
    "CTA2063": ASTMIdType.SERIAL_NUMBER,
    "ANSI_CTA_2063_A": ASTMIdType.SERIAL_NUMBER,
    "CAA_REGISTRATION": ASTMIdType.CAA_REGISTRATION,
    "CAA": ASTMIdType.CAA_REGISTRATION,
    "AESA": ASTMIdType.CAA_REGISTRATION,
    "UTM_UUID": ASTMIdType.UTM_ASSIGNED_UUID,
    "UTM": ASTMIdType.UTM_ASSIGNED_UUID,
    "SESSION_ID": ASTMIdType.SPECIFIC_SESSION_ID,
    "SPECIFIC_SESSION_ID": ASTMIdType.SPECIFIC_SESSION_ID,
}

_STR_TO_UA_TYPE: Dict[str, ASTMUaType] = {
    "HELICOPTER_MULTIROTOR": ASTMUaType.HELICOPTER_MULTIROTOR,
    "MULTIROTOR": ASTMUaType.HELICOPTER_MULTIROTOR,
    "HELICOPTER": ASTMUaType.HELICOPTER_MULTIROTOR,
    "DRONE": ASTMUaType.HELICOPTER_MULTIROTOR,
    "AEROPLANE": ASTMUaType.AEROPLANE,
    "AIRPLANE": ASTMUaType.AEROPLANE,
    "FIXED_WING": ASTMUaType.AEROPLANE,
    "VTOL": ASTMUaType.HYBRID_LIFT_VTOL,
    "HYBRID_LIFT_VTOL": ASTMUaType.HYBRID_LIFT_VTOL,
    "GYROPLANE": ASTMUaType.GYROPLANE,
}


def parse_id_type(value: Any) -> Optional[int]:
    """Interpreta valores numéricos o cadenas de texto para el id_type de ASTM F3411."""
    if value is None:
        return None
    if isinstance(value, (int, float)):
        v_int = int(value)
        return v_int if v_int in ASTM_ID_TYPE_NAMES else None
    if isinstance(value, str):
        v_str = value.strip().upper()
        if v_str.isdigit():
            v_int = int(v_str)
            return v_int if v_int in ASTM_ID_TYPE_NAMES else None
        if v_str in _STR_TO_ID_TYPE:
            return int(_STR_TO_ID_TYPE[v_str])
    return None


def parse_ua_type(value: Any) -> Optional[int]:
    """Interpreta valores numéricos o cadenas de texto para el ua_type de ASTM F3411."""
    if value is None:
        return None
    if isinstance(value, (int, float)):
        v_int = int(value)
        return v_int if 0 <= v_int <= 14 else None
    if isinstance(value, str):
        v_str = value.strip().upper()
        if v_str.isdigit():
            v_int = int(v_str)
            return v_int if 0 <= v_int <= 14 else None
        if v_str in _STR_TO_UA_TYPE:
            return int(_STR_TO_UA_TYPE[v_str])
    return None


# ==============================================================================
# Estructuras de Datos de Identificación
# ==============================================================================
@dataclass
class UAVStandardIdentity:
    """Identificación completa de un dron según ASTM F3411 / EUROCAE ED-282."""
    prim_path: str
    uas_id: str
    id_type: int = int(ASTMIdType.SERIAL_NUMBER)
    ua_type: int = int(ASTMUaType.HELICOPTER_MULTIROTOR)
    operator_id: str = ""
    operator_id_type: int = int(ASTMOperatorIdType.CAA_OPERATOR_ID)
    self_id_desc: str = ""
    model: str = "UAM_aerotaxi"
    is_valid: bool = True
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def id_type_name(self) -> str:
        return ASTM_ID_TYPE_NAMES.get(self.id_type, f"TipoID({self.id_type})")

    @property
    def ua_type_name(self) -> str:
        return ASTM_UA_TYPE_NAMES.get(self.ua_type, f"TipoUA({self.ua_type})")


@dataclass
class FleetValidationReport:
    """Informe consolidado de validación de identidad para toda la flota en el Stage."""
    identities: List[UAVStandardIdentity] = field(default_factory=list)
    conflicts: List[str] = field(default_factory=list)        # Duplicados que generan colisión/conflicto aéreo
    incomplete_errors: List[str] = field(default_factory=list) # Atributos obligatorios faltantes o erróneos
    warnings: List[str] = field(default_factory=list)

    @property
    def is_valid(self) -> bool:
        return len(self.conflicts) == 0 and len(self.incomplete_errors) == 0

    @property
    def has_conflicts(self) -> bool:
        return len(self.conflicts) > 0

    @property
    def uav_count(self) -> int:
        return len(self.identities)


# ==============================================================================
# Funciones de Inspección, Detección y Validación
# ==============================================================================
def is_navsim_drone_prim(prim: Any) -> bool:
    """
    Determina si un Prim de USD representa una aeronave gestionada por NavSim.
    El reconocimiento se basa en la presencia de atributos del espacio de nombres 'NavSim:',
    sin imponer ninguna restricción sobre el nombre del Prim en el árbol de la escena.
    """
    if not prim or not prim.IsValid() or not prim.IsActive():
        return False

    # Descartar luces, cámaras, escenas físicas o planos
    type_name = prim.GetTypeName()
    if type_name in {"DomeLight", "DistantLight", "SphereLight", "RectLight", "Camera", "PhysicsScene", "Plane"}:
        return False

    # Recoger nombres de atributos con prefijo NavSim:
    nav_attrs = {attr.GetName() for attr in prim.GetAttributes() if attr.GetName().startswith("NavSim:")}

    # Descartar prims de vertipuertos o pads a menos que estén explícitamente marcados con ua_type
    path_str = prim.GetPath().pathString
    if ("Vertiport" in path_str or "Pad" in path_str) and "NavSim:ua_type" not in nav_attrs:
        return False

    # Atributos determinantes de aeronave (identidad, categoría, modelo, operador o flag uav)
    drone_markers = {"NavSim:ua_type", "NavSim:model", "NavSim:uav_model", "NavSim:operator_id", "NavSim:is_uav", "NavSim:id"}
    if nav_attrs.intersection(drone_markers):
        return True

    # Fallback para prims de drones históricos con prefijos conocidos
    prim_name = prim.GetName()
    if prim_name.startswith(("UAV_", "uav_", "dron_", "Dron_", "drone_", "Drone_")):
        return True

    return False


def find_navsim_drone_prims(stage: Any) -> List[Any]:
    """
    Recorre el escenario USD y descubre todos los Prims de dron activos basándose
    en sus propiedades NavSim, sin importar su nombre o ubicación en la jerarquía.
    Evita descender a los hijos (rotores, fuselaje, etc.) de un dron ya identificado.
    """
    if not stage:
        return []

    drones = []

    def _traverse(prim):
        if is_navsim_drone_prim(prim):
            drones.append(prim)
            return  # Dron raíz encontrado; no descender a sus submallas/rotores
        for child in prim.GetChildren():
            _traverse(child)

    pseudo_root = stage.GetPseudoRoot()
    if pseudo_root:
        for root in pseudo_root.GetChildren():
            _traverse(root)

    # Ordenar deterministamente por ruta USD
    drones.sort(key=lambda p: str(p.GetPath()))
    return drones


def inspect_prim_identity(prim: Any, fleet_catalog: Optional[Any] = None) -> UAVStandardIdentity:
    """
    Extrae y valida los atributos NavSim / ASTM F3411 de un Prim USD específico:
      1. `NavSim:id` (Obligatorio): Identificador UAS oficial único (`uas_id`).
      2. `NavSim:id_type` (Obligatorio): Tipo de identificador (1=Serial, 2=CAA...).
      3. `NavSim:ua_type` (Obligatorio): Categoría de aeronave (1=Ala fija, 2=Multirrotor...).
      4. `NavSim:operator_id` (Obligatorio): Registro oficial de operador civil.
      5. `NavSim:operator_id_type` (Opcional, defecto=0): Tipo de registro de operador (0=CAA).
      6. `NavSim:self_id_desc` (Opcional, defecto=""): Rol o texto en reposo. En telemetría emite "" si no se define.

    El modelo físico y dinámico no requiere atributo USD; se detecta automáticamente
    mediante la referencia o payload de USD que vincula el asset en `fleet/`.
    """
    prim_path = str(prim.GetPath())
    prim_name = str(prim.GetName())

    errors: List[str] = []
    warnings: List[str] = []

    # 1. Identificador UAS (NavSim:id)
    id_attr = prim.GetAttribute("NavSim:id")
    raw_id = id_attr.Get() if (id_attr and id_attr.IsValid()) else None
    if raw_id is not None and str(raw_id).strip():
        uas_id = str(raw_id).strip()
    else:
        uas_id = prim_name
        errors.append(f"Falta atributo 'NavSim:id' (se usó provisionalmente el nombre del Prim '{prim_name}').")

    # Validación de formato y longitud ASTM F3411 (máximo 20 caracteres UTF-8)
    if len(uas_id) > 20:
        errors.append(
            f"El identificador '{uas_id}' excede los 20 caracteres máximos permitidos por ASTM F3411 (longitud: {len(uas_id)})."
        )
    if not re.match(r"^[A-Za-z0-9_\-]+$", uas_id):
        warnings.append(
            f"El identificador '{uas_id}' contiene caracteres no alfanuméricos estándar."
        )

    # 2. Tipo de Identificador (NavSim:id_type)
    id_type_attr = prim.GetAttribute("NavSim:id_type")
    raw_id_type = id_type_attr.Get() if (id_type_attr and id_type_attr.IsValid()) else None
    parsed_id_type = parse_id_type(raw_id_type)
    if parsed_id_type is not None:
        id_type = parsed_id_type
    else:
        id_type = int(ASTMIdType.SERIAL_NUMBER)
        if raw_id_type is None:
            errors.append(
                "Falta atributo 'NavSim:id_type' (obligatorio según ASTM F3411: 1=SERIAL_NUMBER, 2=CAA_REGISTRATION, 3=UTM_UUID, 4=SESSION_ID)."
            )
        else:
            errors.append(f"Valor 'NavSim:id_type={raw_id_type}' no reconocido en el estándar ASTM F3411.")

    # 3. Categoría de Aeronave (NavSim:ua_type)
    ua_type_attr = prim.GetAttribute("NavSim:ua_type")
    raw_ua_type = ua_type_attr.Get() if (ua_type_attr and ua_type_attr.IsValid()) else None
    parsed_ua_type = parse_ua_type(raw_ua_type)
    if parsed_ua_type is not None:
        ua_type = parsed_ua_type
    else:
        ua_type = int(ASTMUaType.HELICOPTER_MULTIROTOR)
        if raw_ua_type is None:
            errors.append(
                "Falta atributo 'NavSim:ua_type' (obligatorio según ASTM F3411: 2=HELICOPTER_MULTIROTOR, 1=AEROPLANE, 4=VTOL...)."
            )
        else:
            errors.append(f"Valor 'NavSim:ua_type={raw_ua_type}' no reconocido en el estándar ASTM F3411.")

    # 4. Identificador de Operador (NavSim:operator_id)
    op_attr = prim.GetAttribute("NavSim:operator_id")
    raw_op = op_attr.Get() if (op_attr and op_attr.IsValid()) else None
    if raw_op is not None and str(raw_op).strip():
        operator_id = str(raw_op).strip()
    else:
        operator_id = ""
        errors.append(
            "Falta atributo 'NavSim:operator_id' (obligatorio según ASTM F3411 / ED-282 para registro oficial de operador)."
        )

    # 5. Tipo de Operador (NavSim:operator_id_type)
    op_type_attr = prim.GetAttribute("NavSim:operator_id_type")
    raw_op_type = op_type_attr.Get() if (op_type_attr and op_type_attr.IsValid()) else 0
    try:
        operator_id_type = int(raw_op_type) if raw_op_type is not None else 0
    except (ValueError, TypeError):
        operator_id_type = 0

    # 6. Rol / Descripción por Defecto en Reposo (NavSim:self_id_desc - OPCIONAL)
    # El propósito del vuelo en operación activa se deriva dinámicamente de la misión (plan_id / modo de vuelo)
    desc_attr = prim.GetAttribute("NavSim:self_id_desc")
    if not desc_attr or not desc_attr.IsValid():
        desc_attr = prim.GetAttribute("NavSim:description")
    raw_desc = desc_attr.Get() if (desc_attr and desc_attr.IsValid()) else ""
    self_id_desc = str(raw_desc).strip()[:23] if raw_desc else ""

    # 7. Modelo Dinámico (Deducido automáticamente de la referencia o payload original en fleet/)
    model_candidate = None
    try:
        pay = prim.GetMetadata("payload")
        if pay:
            for item in list(getattr(pay, "addedItems", [])) + list(getattr(pay, "prependedItems", [])) + list(getattr(pay, "explicitItems", [])):
                if hasattr(item, "assetPath") and item.assetPath:
                    model_candidate = str(item.assetPath)
                    break
    except Exception:
        pass

    if not model_candidate:
        try:
            ref = prim.GetMetadata("references")
            if ref:
                for item in list(getattr(ref, "addedItems", [])) + list(getattr(ref, "prependedItems", [])) + list(getattr(ref, "explicitItems", [])):
                    if hasattr(item, "assetPath") and item.assetPath:
                        model_candidate = str(item.assetPath)
                        break
        except Exception:
            pass

    # Fallback a compatibilidad previa con atributos o ruta del Prim
    if not model_candidate:
        for attr_name in ("NavSim:model", "NavSim:uav_model", "NavSim:type"):
            attr = prim.GetAttribute(attr_name)
            if attr and attr.IsValid() and attr.Get():
                model_candidate = str(attr.Get()).strip()
                break
    if not model_candidate:
        model_candidate = prim_path

    if fleet_catalog is not None:
        cfg = fleet_catalog.get(model_candidate)
        resolved_model = cfg.model_name
    else:
        resolved_model = model_candidate

    return UAVStandardIdentity(
        prim_path=prim_path,
        uas_id=uas_id,
        id_type=id_type,
        ua_type=ua_type,
        operator_id=operator_id,
        operator_id_type=operator_id_type,
        self_id_desc=self_id_desc,
        model=resolved_model,
        is_valid=(len(errors) == 0),
        errors=errors,
        warnings=warnings,
    )


def validate_fleet_prims(prims: List[Any], fleet_catalog: Optional[Any] = None) -> FleetValidationReport:
    """
    Inspecciona y valida una lista de Prims UAV en la escena.
    Detecta de forma exhaustiva:
      1. Conflicto crítico de duplicidad de identificadores UAS en el aire.
      2. Errores de identificación incompleta o inválida en cada dron.
    """
    report = FleetValidationReport()
    ids_to_paths: Dict[str, List[str]] = {}

    for prim in prims:
        identity = inspect_prim_identity(prim, fleet_catalog=fleet_catalog)
        report.identities.append(identity)

        norm_id = identity.uas_id.strip().upper()
        if norm_id:
            ids_to_paths.setdefault(norm_id, []).append(identity.prim_path)

        for err in identity.errors:
            report.incomplete_errors.append(f"[{identity.prim_path}] {err}")
        for warn in identity.warnings:
            report.warnings.append(f"[{identity.prim_path}] {warn}")

    # Comprobar duplicidad de identificadores (conflicto aéreo directo)
    for uas_id_norm, paths in ids_to_paths.items():
        if len(paths) > 1:
            conflict_msg = (
                f"🚨 CONFLICTO CRÍTICO EN EL AIRE: El identificador '{uas_id_norm}' está duplicado "
                f"en {len(paths)} aeronaves: {paths}. En el estándar ASTM F3411 / ED-282 dos aeronaves "
                f"no pueden emitir el mismo identificador simultáneamente."
            )
            report.conflicts.append(conflict_msg)

    return report
