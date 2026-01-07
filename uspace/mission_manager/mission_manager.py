from msgs.mission_manager_msgs import MissionMsg


class MissionManager:
    def __init__(self):
        self.id: str
        self.name:str
        self.missions = []
        self.msg = MissionMsg()

    def add_mission(self, mission):
        self.missions.append(mission)

    def get_missions(self):
        return self.missions

    def clear_missions(self):
        self.missions = []