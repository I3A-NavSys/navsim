from .waypoint_msg import WaypointMsg

class FlightPlanMsg:
    def __init__(self) -> None:
        self.plan_id: int
        self.uav_id: str
        self.operator_id: str
        self.route: WaypointMsg
        self.mode: str
        self.radius: float
        self.priority: int
