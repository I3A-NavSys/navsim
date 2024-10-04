# Adding root 'project' folder to sys.path
import sys
import os
# This line will add to the python list of paths to look for modules the path to the project root
project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))

if project_root_path not in sys.path:
    sys.path.append(project_root_path)

import omni.ext
import omni.ui as ui
from omni.ui import color as cl

from omni.isaac.ui.ui_utils import dropdown_builder
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.ui.element_wrappers import *

import carb.events

import pickle   # Serialization
import base64   # Parsing to string

import omni.kit.app

from uspace.flightplan.Waypoint   import Waypoint
from uspace.flightplan.FlightPlan import FlightPlan

import omni.physx
import omni.timeline

from ov_utils import extensions_utils as ext_utils

class NavsimOperatorCmdExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        # print("[REMOTE COMMAND ext] startup")

        # Getting the simulation current time
        self.current_time = 0
        self.physx_interface = omni.physx.get_physx_interface()
        self.physics_timer_callback = self.physx_interface.subscribe_physics_step_events(self.physics_timer_callback_fn)

        self.timeline = omni.timeline.get_timeline_interface()
        self.event_timer_callback = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.timeline_timer_callback_fn
        )

        # Get the bus event stream
        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()

        self.app_interface = omni.kit.app.get_app_interface()

        # Create the window
        self._window = ui.Window("REMOTE FLIGHTPLAN", width=400, height=200)
        with self._window.frame:

            with ui.VStack(spacing=10, height=0):
                ui.Spacer(height=10)


                # UAV selector dropdown
                self.UAV_selector_dropdown = ext_utils.build_uav_selector()

                with ui.HStack(spacing=10):
       
                    ui.Button("delay", enable=False, width=50,
                              style={"background_color":cl.grey, 
                                    "color":cl.white, 
                                    "margin": 0})
                    
                    self.delay_FF = ui.FloatField(width=50, tooltip="Delay related to sending time")
                    self.delay_FF.model.set_value(5.0)
                    self.delay_FF.precision = 2
                    
                with ui.HStack(spacing=20):
                    ui.Button("TUTO4", clicked_fn = self.tuto4)
                    ui.Button("3x3CUBES", clicked_fn = self.cubes3x3)
                    ui.Button("REL_ABS", clicked_fn = self.rel_abs_test)
                    ui.Button("SMOOTH", clicked_fn = self.smooth_test)
                            


    def on_shutdown(self):
        # print("[REMOTE COMMAND ext] shutdown")
        self.stop_update_plot = True
        pass



    def physics_timer_callback_fn(self, step_size:int):
        self.current_time += step_size



    def timeline_timer_callback_fn(self, event):
        self.current_time = 0



    def tuto4(self):
        if self.UAV_selector_dropdown.get_selection() is None:
            raise Exception("[REMOTE FLIGHTPLAN ext] No drone selected")
        
        drone = ext_utils.get_prim_by_name(self.UAV_selector_dropdown.get_selection())
        
        delay = self.delay_FF.model.get_value_as_float()

        self.fp = FlightPlan()
        self.fp.radius = 2

        self.fp.SetWaypoint(label="wp1L", pos=[-190, -119, 48.1])
        self.fp.SetWaypoint(label="wp1P", time=5, pos=[-190, -119, 48.1])
        self.fp.SetWaypoint(label="wp1", pos=[-190, -119, 70])
        self.fp.SetWaypoint(label="wp2", pos=[-90, -19, 70])
        self.fp.SetWaypoint(label="wp3", pos=[-90, 181, 70])
        self.fp.SetWaypoint(label="wp4", pos=[110, 181, 70])
        self.fp.SetWaypoint(label="wp5", pos=[110, -119, 70])
        self.fp.SetWaypoint(label="wp6", pos=[-152, -106, 70])
        self.fp.SetWaypoint(label="wp6L", pos=[-152, -106, 49.1])
        self.fp.SetWaypoint(label="wp6P", time=18, pos=[-152, -106, 49.1])

        self.fp.SetUniformVelocity("wp1P", 2)
        self.fp.SetUniformVelocity("wp1", 8)
        self.fp.SetUniformVelocity("wp2", 8)
        self.fp.SetUniformVelocity("wp3", 8)
        self.fp.SetUniformVelocity("wp4", 8)
        self.fp.SetUniformVelocity("wp5", 8)
        self.fp.SetUniformVelocity("wp6", 2)
        self.fp.SetUniformVelocity()

        ang_vel = 0.1
        lin_acel = 0.4

        self.fp.SmoothWPDuration('wp1P',ang_vel, lin_acel)
        self.fp.SmoothWPDuration('wp1',ang_vel, lin_acel)
        self.fp.SmoothWPSpeed('wp2',ang_vel)
        self.fp.SmoothWPSpeed('wp3',ang_vel)
        self.fp.SmoothWPSpeed('wp4',ang_vel)
        self.fp.SmoothWPSpeed('wp5',ang_vel)
        self.fp.SmoothWPDuration('wp6',ang_vel, lin_acel)
        self.fp.SmoothWPDuration('wp6L',ang_vel, lin_acel)

        self.fp.Postpone(self.current_time + delay)

        self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(drone.GetPath()))

        serialized_fp = base64.b64encode(pickle.dumps(self.fp)).decode('utf-8')
        self.event_stream.push(self.UAV_EVENT, payload={"method": "eventFn_FlightPlan", "fp": serialized_fp})



    def cubes3x3(self):
        if self.UAV_selector_dropdown.get_selection() is None:
            raise Exception("[REMOTE FLIGHTPLAN ext] No drone selected")
        
        drone = ext_utils.get_prim_by_name(self.UAV_selector_dropdown.get_selection())
        
        delay = self.delay_FF.model.get_value_as_float()

        # Crear plan de vuelo
        self.fp = FlightPlan()
        self.fp.radius = 2
        self.fp.SetWaypoint(label='WPI', time=0,  pos=[0.0, 0.0, 1.0])
        self.fp.SetWaypoint(label='WPIP', time=5,  pos=[0.0, 0.0, 2.0])
        self.fp.SetWaypoint(label='WP1', time=15,  pos=[0.0, 10.0, 2.0])
        self.fp.SetWaypoint(label='WP2', time=25,  pos=[-10.0, 10.0, 2.0])
        self.fp.SetWaypoint(label='WP3', time=35,  pos=[-10.0, 0.0, 2.0])
        self.fp.SetWaypoint(label='WP4', time=45,  pos=[.0, -10.0, 2.0])
        self.fp.SetWaypoint(label='WP5', time=55,  pos=[10.0, 10.0, 2.0])
        self.fp.SetWaypoint(label='WP6', time=65,  pos=[10.0, 0.0, 2.0])
        self.fp.SetWaypoint(label='WP7', time=75,  pos=[10.0, -10.0, 2.0])
        self.fp.SetWaypoint(label='WPF', time=85,  pos=[-10.0, -10.0, 2.0])
        self.fp.SetWaypoint(label='WPFP', time=90,  pos=[-10.0, -10.0, 1.0])

        self.fp.SetUniformVelocity()
        
        self.fp.SmoothWPSpeed("WPIP", 0.3)
        self.fp.SmoothWPSpeed("WP1", 0.3)
        self.fp.SmoothWPSpeed("WP2", 0.3)
        self.fp.SmoothWPSpeed("WP3", 0.3)
        self.fp.SmoothWPSpeed("WP4", 0.3)
        self.fp.SmoothWPSpeed("WP5", 0.8)
        self.fp.SmoothWPSpeed("WP7", 0.3)
        self.fp.SmoothWPSpeed("WPF", 0.3)

        self.fp.Postpone(self.current_time + delay)

        self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(drone.GetPath()))

        serialized_fp = base64.b64encode(pickle.dumps(self.fp)).decode('utf-8')
        self.event_stream.push(self.UAV_EVENT, payload={"method": "eventFn_FlightPlan", "fp": serialized_fp})



    def rel_abs_test(self):
        if self.UAV_selector_dropdown.get_selection() is None:
            raise Exception("[REMOTE FLIGHTPLAN ext] No drone selected")
        
        drone = ext_utils.get_prim_by_name(self.UAV_selector_dropdown.get_selection())

        delay = self.delay_FF.model.get_value_as_float()

        # Creamos plan de vuelo
        self.fp = FlightPlan()
        self.fp.radius = 2
        self.fp.SetWaypoint(label="wp1", time=0,  pos=[  0.0,  0.0,  1.0 ])
        self.fp.SetWaypoint(label="wp2", time=10,  pos=[  0.0,  0.0,  2.0 ])
        self.fp.SetWaypoint(label="wp3", time=20, pos=[ 0.0,  10.0,  2.0 ])
        self.fp.SetWaypoint(label="wp4", time=30, pos=[ 0.0,  10.0,  1.0 ])

        self.fp.SetUniformVelocity()

        self.fp.Postpone(self.current_time + delay)

        # Comunicamos plan de vuelo
        self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(drone.GetPath()))
        serialized_fp = base64.b64encode(pickle.dumps(self.fp)).decode('utf-8')
        self.event_stream.push(self.UAV_EVENT, payload={"method": "eventFn_FlightPlan", "fp": serialized_fp})



    def smooth_test(self):
        if self.UAV_selector_dropdown.get_selection() is None:
            raise Exception("[REMOTE FLIGHTPLAN ext] No drone selected")
        
        drone = ext_utils.get_prim_by_name(self.UAV_selector_dropdown.get_selection())

        delay = self.delay_FF.model.get_value_as_float()

        # Creamos plan de vuelo
        self.fp = FlightPlan()
        self.fp.radius = 2
        self.fp.SetWaypoint(label="wp1", time=0,  pos=[  0.0,  0.0,  1.0 ])
        self.fp.SetWaypoint(label="wp2", time=5,  pos=[  0.0,  0.0,  2.0 ])
        self.fp.SetWaypoint(label="wp3", time=15, pos=[ 10.0,  0.0,  2.0 ])
        self.fp.SetWaypoint(label="wp4", time=25, pos=[ 10.0,  -10.0,  2.0 ])
        self.fp.SetWaypoint(label="wp5", time=30, pos=[ 10.0,  -10.0,  1.0 ])

        self.fp.SetUniformVelocity()

        self.fp.SmoothWPSpeed("wp3", 0.11)

        self.fp.Postpone(self.current_time + delay)

        # Comunicamos plan de vuelo
        self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(drone.GetPath()))
        serialized_fp = base64.b64encode(pickle.dumps(self.fp)).decode('utf-8')
        self.event_stream.push(self.UAV_EVENT, payload={"method": "eventFn_FlightPlan", "fp": serialized_fp})  