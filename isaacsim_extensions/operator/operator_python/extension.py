import os


import omni.ext
from isaacsim.gui.components import ui
from omni.ui import color
import logging


from navsim_utils.sim_utils import UAVStatus
from navsim_utils.extensions_utils import ExtensionUtils, MinimalComboBoxModel
from navsim_utils.paths_utils import project_root_path, ui_icons_path
from .ui_builder import UIBuilder


class Operator(omni.ext.IExt):
    def on_startup(self, ext_id) -> None:
        self.init_vars()
        self.ui_builder.build_ui()

    def on_shutdown(self) -> None:
        pass

    def init_vars(self) -> None:
        # Logging
        self.logger = logging.getLogger("NavSim.Operator")
        
        # Utils
        self.extension_utils = ExtensionUtils()
        
        # Main data
        self.operators = {
            "Operator_1": {
                "uavs": {
                    "UAV_1": {
                        "type": "AEROTAXI",           # AEROTAXI / QUADCOPTER
                        "status": UAVStatus.IDLE,         # BUSY / IDLE / OFFLINE
                        "mission": "mission_1",        # unique mission identifier
                        "time": "12:38:53",           # UAV update current time
                        "battery": 100,        # not used yet
                        "location": [150, 200, 1.75],       # current UAV location (x,y,z)
                    },
                    "UAV_2": {
                        "type": "QUADCOPTER",           # AEROTAXI / QUADCOPTER
                        "status": UAVStatus.BUSY,         # BUSY / IDLE / OFFLINE
                        "mission": "mission_2",        # unique mission identifier
                        "time": "13:12:27",           # UAV update current time
                        "battery": 80,        # not used yet
                        "location": [350, -100, 1.75],       # current UAV location (x,y,z)
                    }
                }, 
                "missions": {
                    # "Mission_1": {
                    #     "client_id": None,      # unique client identifier
                    #     "uav_id": None,         # assigned UAV identifier
                    #     "flightplan": None,     # flightplan object
                    #     "status": None,         # PENDING / IN_PROGRESS / COMPLETED / FAILED
                    #     "start_time": None,     # mission start time
                    #     "end_time": None        # mission end time
                    # }
                },
                "statistics": {
                    # "total_missions": 0,
                    # "successful_missions": 0,
                    # "failed_missions": 0,
                    # "average_completion_time": 0.0
                }
            },
            "Operator_2": {
                "uavs": {}, 
                "missions": {}, 
                "statistics": {}
            },
        }

        # UI Builder
        self.ui_builder = UIBuilder(self.extension_utils, self.operators)


