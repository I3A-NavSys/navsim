class Mission:
    def __init__(
        self, 
        id, 
        mission_type, 
        stop_list, 
        stop_times, 
        uav_operator_id, 
        landing_time,
        status
    ):
        self.uav_operator_id = uav_operator_id
        self.id = id
        self.mission_type = mission_type
        self.stop_list = stop_list
        self.stop_times = stop_times
        self.landing_time = landing_time
        self.status = status