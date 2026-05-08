class MissionType:
    DELIVERY = "DELIVERY"
    PASSENGER_TRANSPORT = "PASSENGER_TRANSPORT"

class MissionStatus:
    PENDING = "PENDING"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    CANCELLED = "CANCELLED"

class UAVStatus:
    AVAILABLE = "AVAILABLE"
    BUSY = "BUSY"
    PENDING_ASSIGNMENT = "PENDING_ASSIGNMENT"
    OUT_OF_SERVICE = "OUT_OF_SERVICE"

class PadStatus:
    OPERATIVE = "OPERATIVE"
    OUT_OF_SERVICE = "OUT_OF_SERVICE"

class CancellationReason:
    UNSUPPORTED_MISSION_TYPE = "UNSUPPORTED_MISSION_TYPE"
    NO_AVAILABLE_UAV = "NO_AVAILABLE_UAV"
    FLIGHTPLAN_IN_THE_PAST = "FLIGHTPLAN_IN_THE_PAST"
    NO_AVAILABLE_PAD = "NO_AVAILABLE_PAD"
    MAIN_PAD_OCCUPIED = "MAIN_PAD_OCCUPIED"
    NO_AVAILABLE_ROUTE = "NO_AVAILABLE_ROUTE"

class Topics:
    # General Topics
    CANCEL_MISSION = "general/missions/cancellation"

    # USpace Manager Topics
    UAV_OPERATOR_REGISTER = "airspace/operators/register/uav"
    VERTIPORT_OPERATOR_REGISTER = "airspace/operators/register/vertiport"
    REQUEST_ROUTE = "airspace/operators/routes/request"
    RECEIVE_TAKEOFF_FLIGHTPLAN = "airspace/operators/routes/reception/takeoff"
    RECEIVE_LANDING_FLIGHTPLAN = "airspace/operators/routes/reception/landing"
    REQUEST_VERTIPORT_OPERATOR_LIST = "airspace/mission_managers/request/vertiport_operator_list"
    REQUEST_UAV_OPERATOR_LIST = "airspace/mission_managers/request/uav_operator_list"

    # UAV Operator Topics
    MISSION_UAV_SERVICE = "uav_operators/missions/request"
    PRIVATE_VERTIPORT_TAKEOFF = "uav_operators/missions/routes/takeoff"
    PRIVATE_VERTIPORT_LANDING = "uav_operators/missions/routes/landing"

    # Vertiport Operator Topics
    MISSION_VERTIPORT_SERVICE = "vertiport_operators/missions/request"
    REQUEST_VERTIPORT_INFO = "vertiport_operators/info/request"

    # Mission Manager Topics
    MISSION_STATUS_UPDATE = "mission_managers/missions/status_update"
