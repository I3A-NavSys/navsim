from enum import Enum


class MissionType(Enum):
    DELIVERY = "DELIVERY"
    PASSENGER_TRANSPORT = "PASSENGER_TRANSPORT"

class MissionStatus(Enum):
    PENDING = "PENDING"
    IN_PROGRESS = "IN_PROGRESS"
    COMPLETED = "COMPLETED"
    CANCELLED = "CANCELLED"

class UAVStatus(Enum):
    AVAILABLE = "AVAILABLE"
    OCCUPIED = "OCCUPIED"
    OUT_OF_SERVICE = "OUT_OF_SERVICE"

class Topics(Enum):
    # USpace Manager Topics
    UAV_OPERATOR_REGISTER = "airspace/operators/register/uav"
    VERTIPORT_OPERATOR_REGISTER = "airspace/operators/register/vertiport"
    REQUEST_VERTIPORT_OPERATOR_LIST = "airspace/mission_managers/request/vertiport_operator_list"
    REQUEST_UAV_OPERATOR_LIST = "airspace/mission_managers/request/uav_operator_list"

    # UAV Operator Topics
    MISSION_UAV_SERVICE = "uav_operators/missions/request"
