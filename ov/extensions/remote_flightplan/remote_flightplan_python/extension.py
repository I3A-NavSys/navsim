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

        # List of manipulable prims (usually drones)
        self.manipulable_prims = []

        # Get the bus event stream
        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()

        self.app_interface = omni.kit.app.get_app_interface()

        # Create the window
        self._window = ui.Window("REMOTE FLIGHTPLAN", width=400, height=200)
        with self._window.frame:

            with ui.VStack(spacing=10):
                ui.Spacer(height=10)


                # UAV selector dropdown
                with ui.HStack(height=25):
                    # Dropdown selector
                    self.drone_selector_dropdown = DropDown(
                        "UAV", "Select the drone you want to send commands", 
                        self.get_manipulable_prims)
                    self.drone_selector_dropdown.enabled = False

                    # Button to refresh manipulable drones
                    self.refresh_selector_button = ui.Button(
                        "REFRESH", 
                        clicked_fn=self.refresh_drone_selector, 
                        height=15,
                        width=80)

                with ui.HStack(height=20, spacing=10):
       
                    ui.Button(
                        "delay", enable=False,
                        width=50,
                        style={
                            "background_color":cl.grey, 
                            "color":cl.white, 
                            "margin": 0})
                    self.delay_FF = ui.FloatField(
                        width=50,
                        tooltip="Delay related to sending time")
                    self.delay_FF.model.set_value(5.0)
                    self.delay_FF.precision = 2
                    
                with ui.HStack(height=50, spacing=20):
                    # fp button
                    ui.Button(
                        "TUTO4",
                        clicked_fn = self.tuto4
                    )

                    ui.Button(
                        "3x3CUBES",
                        clicked_fn = self.cubes3x3
                    )

                    ui.Button(
                        "REL_ABS",
                        clicked_fn = self.rel_abs_test
                    )

                    ui.Button(
                        "SMOOTH",
                        clicked_fn = self.smooth_test
                    )



    def on_update(self, time, dt):
        print("[REMOTE COMMAND ext] update")
        pass



    def get_manipulable_prims(self, prim=None):
            # Needed list containing the names of the manipulable prims for the dropdown selector
            manipulable_prims = []

            # First iteration
            if prim is None:
                stage = get_current_stage()
                prim = stage.GetPseudoRoot()

                self.manipulable_prims.clear()

            # Check current prim
            if prim.GetAttribute("NavSim:Manipulable").IsValid() and prim.GetAttribute("NavSim:Manipulable").Get():
                manipulable_prims.append(prim.GetName())
                self.manipulable_prims.append(prim)
            
            else:
                # Check prim's children
                for child in prim.GetAllChildren():
                    # First check children
                    if len(child.GetAllChildren()) > 0:
                        manipulable_prims += self.get_manipulable_prims(child)
                    
                    # Then check current child
                    elif child.GetAttribute("NavSim:Manipulable").IsValid() and child.GetAttribute("NavSim:Manipulable").Get():
                        manipulable_prims.append(child.GetName())
                        self.manipulable_prims.append(prim)

            return manipulable_prims
        


    def refresh_drone_selector(self):
        self.drone_selector_dropdown.enabled = True
        self.drone_selector_dropdown.repopulate()



    def on_shutdown(self):
        # print("[REMOTE COMMAND ext] shutdown")
        self.stop_update_plot = True
        pass



    def physics_timer_callback_fn(self, step_size:int):
        self.current_time += step_size



    def timeline_timer_callback_fn(self, event):
        self.current_time = 0



    def tuto4(self):
        delay = self.delay_FF.model.get_value_as_float()

        # Crear waypoints
        wp1L = Waypoint(label="wp1L", pos=[-190, -119, 48.1])
        wp1P = Waypoint(label="wp1P", pos=[-190, -119, 48.1])
        wp1 = Waypoint(label="wp1", pos=[-190, -119, 70])
        wp2 = Waypoint(label="wp2", pos=[-90, -19, 70])
        wp3 = Waypoint(label="wp3", pos=[-90, 181, 70])
        wp4 = Waypoint(label="wp4", pos=[110, 181, 70])
        wp5 = Waypoint(label="wp5", pos=[110, -119, 70])
        wp6 = Waypoint(label="wp6", pos=[-152, -106, 70])
        wp6L = Waypoint(label="wp6L", pos=[-152, -106, 49.1])
        wp6P = Waypoint(label="wp6P", pos=[-152, -106, 49.1])

        self.fp = FlightPlan()
        self.fp.radius = 2
        self.fp.AppendWaypoint(wp1L)
        wp1P.t = self.fp.FinishTime() + 5
        self.fp.SetWaypoint(wp1P)
        self.fp.AppendWaypoint(wp1)
        self.fp.AppendWaypoint(wp2)
        self.fp.AppendWaypoint(wp3)
        self.fp.AppendWaypoint(wp4)
        self.fp.AppendWaypoint(wp5)
        self.fp.AppendWaypoint(wp6)
        self.fp.AppendWaypoint(wp6L)
        wp6P.t = self.fp.FinishTime() + 5
        self.fp.SetWaypoint(wp6P)

        self.fp.SetUniformVelocity("wp1P", 2)
        self.fp.SetUniformVelocity("wp1", 8)
        self.fp.SetUniformVelocity("wp2", 8)
        self.fp.SetUniformVelocity("wp3", 8)
        self.fp.SetUniformVelocity("wp4", 8)
        self.fp.SetUniformVelocity("wp5", 8)
        self.fp.SetUniformVelocity("wp6", 2)

        ang_vel = 0.1
        lin_acel = 0.4

        self.fp.SmoothWPDuration('wp1P',ang_vel,lin_acel)
        self.fp.SmoothWPDuration('wp1',ang_vel,lin_acel)
        self.fp.SmoothWPSpeed('wp2',ang_vel)
        self.fp.SmoothWPSpeed('wp3',ang_vel)
        self.fp.SmoothWPSpeed('wp4',ang_vel)
        self.fp.SmoothWPSpeed('wp5',ang_vel)
        self.fp.SmoothWPDuration('wp6',ang_vel,lin_acel)
        self.fp.SmoothWPDuration('wp6L',ang_vel,lin_acel)

        self.fp.RescheduleAt(self.current_time + delay)

        try:
            drone = self.manipulable_prims[self.drone_selector_dropdown.get_selection_index()]
        except:
            print("[REMOTE COMMAND ext] No drone selected")
            return

        self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(drone.GetPath()))

        serialized_fp = base64.b64encode(pickle.dumps(self.fp)).decode('utf-8')
        self.event_stream.push(self.UAV_EVENT, payload={"method": "eventFn_FlightPlan", "fp": serialized_fp})



    def cubes3x3(self):
        delay = self.delay_FF.model.get_value_as_float()

        wpI = Waypoint(label='WPI', t=0,  pos=[0.0, 0.0, 1.0])
        wpIP = Waypoint(label='WPIP', t=5,  pos=[0.0, 0.0, 1.0])
        wp1 = Waypoint(label='WP1', t=15,  pos=[0.0, 10.0, 1.0])
        wp2 = Waypoint(label='WP2', t=25,  pos=[0.0, 20.0, 1.0])
        wp2 = Waypoint(label='WP2', t=25,  pos=[-10.0, 10.0, 1.0])
        wp3 = Waypoint(label='WP3', t=35,  pos=[-10.0, 0.0, 1.0])
        wp4 = Waypoint(label='WP4', t=45,  pos=[.0, -10.0, 1.0])
        wp5 = Waypoint(label='WP5', t=55,  pos=[10.0, 10.0, 1.0])
        wp6 = Waypoint(label='WP6', t=65,  pos=[10.0, 0.0, 1.0])
        wp7 = Waypoint(label='WP7', t=75,  pos=[10.0, -10.0, 1.0])
        wpF = Waypoint(label='WPF', t=85,  pos=[-10.0, -10.0, 1.0])
        wpFP = Waypoint(label='WPP', t=90,  pos=[-10.0, -10.0, 1.0])

        # Crear plan de vuelo
        self.fp = FlightPlan([wpI, wpIP, wp1, wp2, wp3, wp4, wp5, wp6, wp7, wpF, wpFP])
        self.fp.radius = 2

        self.fp.SetUniformVelocity("WP1", 1)
        self.fp.SetUniformVelocity("WP2", 2)
        self.fp.SetUniformVelocity("WP3", 2)
        self.fp.SetUniformVelocity("WP4", 2)
        self.fp.SetUniformVelocity("WP5", 2)
        self.fp.SetUniformVelocity("WP6", 2)
        self.fp.SetUniformVelocity("WP7", 2)
        self.fp.SetUniformVelocity("WPF", 1)
        
        self.fp.ConnectWPs()

        self.fp.RescheduleAt(self.current_time + delay)

        try:
            drone = self.manipulable_prims[self.drone_selector_dropdown.get_selection_index()]
        except:
            print("[REMOTE COMMAND ext] No drone selected")
            return

        self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(drone.GetPath()))

        serialized_fp = base64.b64encode(pickle.dumps(self.fp)).decode('utf-8')
        self.event_stream.push(self.UAV_EVENT, payload={"method": "eventFn_FlightPlan", "fp": serialized_fp})



    def rel_abs_test(self):

        try:
            drone = self.manipulable_prims[self.drone_selector_dropdown.get_selection_index()]
        except:
            print("[REMOTE COMMAND ext] No drone selected")
            return
        
        print("[REMOTE COMMAND ext] Drone selected is: ", drone.GetPath())


        delay = self.delay_FF.model.get_value_as_float()

        # Creamos plan de vuelo
        wp1 = Waypoint("wp1", t=10,  pos=[  0.0,  0.0,  1.0 ])
        wp2 = Waypoint("wp2", t=20,  pos=[  0.0,  0.0,  2.0 ])
        wp3 = Waypoint("wp3", t=30, pos=[ 0.0,  10.0,  2.0 ])
        wp4 = Waypoint("wp4", t=40, pos=[ 0.0,  10.0,  1.0 ])
        # wp5 = Waypoint("wp5", t=50, pos=[ 10.0,  -10.0,  1.0 ])
        self.fp = FlightPlan([wp1, wp2, wp3, wp4])
        self.fp.radius = 2
        self.fp.SetUniformVelocity()

        self.fp.RescheduleAt(self.current_time + delay)

        # Comunicamos plan de vuelo
        self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(drone.GetPath()))
        serialized_fp = base64.b64encode(pickle.dumps(self.fp)).decode('utf-8')
        self.event_stream.push(self.UAV_EVENT, payload={"method": "eventFn_FlightPlan", "fp": serialized_fp})



    def smooth_test(self):

        try:
            drone = self.manipulable_prims[self.drone_selector_dropdown.get_selection_index()]
        except:
            print("[REMOTE COMMAND ext] No drone selected")
            return
        
        print("[REMOTE COMMAND ext] Drone selected is: ", drone.GetPath())

        delay = self.delay_FF.model.get_value_as_float()

        # Creamos plan de vuelo
        wp1 = Waypoint("wp1", t=10,  pos=[  0.0,  0.0,  1.0 ])
        wp2 = Waypoint("wp2", t=15,  pos=[  0.0,  0.0,  2.0 ])
        wp3 = Waypoint("wp3", t=25, pos=[ 10.0,  0.0,  2.0 ])
        wp4 = Waypoint("wp4", t=35, pos=[ 10.0,  -10.0,  2.0 ])
        wp5 = Waypoint("wp5", t=40, pos=[ 10.0,  -10.0,  1.0 ])
        self.fp = FlightPlan([wp1, wp2, wp3, wp4, wp5])
        self.fp.radius = 2

        self.fp.SetUniformVelocity()

        self.fp.SmoothWPSpeed("wp3", 0.1)

        self.fp.Postpone(self.current_time + delay)

        # Comunicamos plan de vuelo
        self.UAV_EVENT = carb.events.type_from_string("NavSim." + str(drone.GetPath()))
        serialized_fp = base64.b64encode(pickle.dumps(self.fp)).decode('utf-8')
        self.event_stream.push(self.UAV_EVENT, payload={"method": "eventFn_FlightPlan", "fp": serialized_fp})  