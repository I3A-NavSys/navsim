from enum import Enum


class MissionType(Enum):
    DELIVERY = "DELIVERY"
    PASSENGER_TRANSPORT = "PASSENGER_TRANSPORT"

class Topics(Enum):
    UAV_REGISTER = "airspace/operators/register/uav"
    VERTIPORT_REGISTER = "airspace/operators/register/vertiport"
    REQUEST_VERTIPORT_OPERATOR_LIST = "airspace/mission_managers/request/vertiport_operator_list"
    REQUEST_UAV_OPERATOR_LIST = "airspace/mission_managers/request/uav_operator_list"