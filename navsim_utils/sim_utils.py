from datetime import datetime, timedelta
import pyproj


class TypeSender:
    USPACE_MANAGER = "uspace_manager"
    USPACE_CLIENT = "uspace_client"
    OPERATOR_UAV = "operator_uav"
    OPERATOR_VERTIPORT = "operator_vertiport"
    FLIGHTPLAN_GENERATOR = "flight_plan_generator"
    COMMAND_GENERATOR = "command_generator"
    SINGLE_UAV = "single_uav"

class TypeMessage:
    EXTENSSION_ON_OFF = "extension_on_off"
    CMD_FP_REQUEST = "cmd_fp_request"
    USPACE = "uspace"

class RequestState:
    CANCELLED = "Cancelled"
    PENDING = "Pending"
    IN_PROGRESS = "In progress"
    COMPLETED = "Completed"

class UAVState:
    IDLE = "idle"
    BUSY = "busy"
    DEAD = "dead"

class TimeManager:
    def __init__(self):
        self.start_real_time = None
        self.time_offset = 0.0

    def start(self):
        self.start_real_time = datetime.now()

    def stop(self):
        self.start_real_time = None
        self.time_offset = 0.0

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

class GeospatialManager:
    def __init__(self):
        self.origin = None
        self.transformer = None

    def set_origin(self, lat, lon, alt=0):
        self.origin = (lat, lon, alt)
        self.transformer = pyproj.Transformer.from_crs(
            "EPSG:4979",
            "+proj=aeqd +lat_0={} +lon_0={} +x_0=0 +y_0=0".format(lat, lon),
            always_xy=True
        )
    
    def geo_to_sim(self, lat, lon, alt):
        x, y, z = self.transformer.transform(lon, lat, alt)
        return (x, y, z)  # IsaacSim coordinates
    
    def sim_to_geo(self, x, y, z):
        lon, lat, alt = self.transformer.transform(x, y, z, direction="INVERSE")
        return (lat, lon, alt)
    