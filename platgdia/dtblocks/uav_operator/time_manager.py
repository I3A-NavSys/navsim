from datetime import datetime, timedelta

class TimeManager:
    def __init__(self):
        self.start_real_time = None
        self.time_offset = 0.0
        self.current_sim_time = 0.0
        self.current_real_time = None

    def start(self):
        self.start_real_time = datetime.now()

    def stop(self):
        self.start_real_time = None
        self.time_offset = 0.0
        self.current_sim_time = 0.0
        self.current_real_time = None

    def pause(self):
        self.start_pause_time = datetime.now()

    def resume(self):
        end_pause_time = datetime.now()
        pause_duration = (end_pause_time - self.start_pause_time).total_seconds()
        self.time_offset += pause_duration

    def real_to_sim(self, real_time=None):
        if real_time is None:   time = datetime.now()
        else:                   time = real_time
        
        sim_time = (time - self.start_real_time).total_seconds() - self.time_offset
        return sim_time
    
    def sim_to_real(self, sim_time):
        real_time = self.start_real_time + timedelta(seconds=(sim_time + self.time_offset))
        return real_time