class NavigationReportMsg:
    def __init__(self, plan_id: int, uav_id: str, operator_id: str, fp_aborted: bool, fp_running: bool, 
                 fp_completed: bool, current_wp: int, time: float):
        self.plan_id = plan_id
        self.uav_id = uav_id
        self.operator_id = operator_id
        self.fp_aborted = fp_aborted
        self.fp_running = fp_running
        self.fp_completed = fp_completed
        self.current_wp = current_wp
        self.time = time