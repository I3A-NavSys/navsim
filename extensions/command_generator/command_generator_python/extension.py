import pickle   # Serialization
import base64   # Parsing to string
import sys, os
import torch


import omni.ext
from isaacsim.gui.components.ui_utils import ui
from isaacsim.gui.components.element_wrappers import *
import carb.events
import omni.timeline
import omni.physx
from omni.isaac.core.prims import RigidPrimView


from uspace.flight_plan.command import Command
from navsim_utils.extensions_utils import ExtensionUtils
# from fleet.uav_ia_control import UAVcontrol
from fleet.uav_matrix_control import UAVcontrol
# from fleet.uav_matrix_control_quadcopter import UAVcontrol


file_path = os.path.dirname(__file__)
project_root_path = os.path.abspath(os.path.join(file_path, '../../..'))
if project_root_path not in sys.path:
    sys.path.append(project_root_path)


class CommandGenerator(omni.ext.IExt):

    def on_startup(self, ext_id):
        self.init_vars()
        self.build_ui()

    def on_shutdown(self):
        self.on_physics_step_sub = None
        self.on_stop_sub = None
        self.on_play_sub = None

    def init_vars(self):
        self.rigid_prim_view = None
        self.uav_control = None
        
        self.navsim_utils = ExtensionUtils()

        self.physx_interface = omni.physx.get_physx_interface()
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_on_step_events(self.on_physics_step, True, 0)

        self.timeline = omni.timeline.get_timeline_interface()
        self.on_stop_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), self.on_stop
        )
        self.on_play_sub = self.timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY), self.on_play
        )

        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()

        self.is_sim_played = False
        self.current_time = 0
        self.uavs = {}

    def on_physics_step(self, step_size:int):
        if self.is_sim_played:
            self.current_time += step_size
            self.uav_control.update(self.current_time, self.uavs, step_size)

    def on_stop(self, event):
        self.current_time = 0
        self.is_sim_played = False
        self.rigid_prim_view = None
        self.uav_control = None

    def on_play(self, event):
        if not self.is_sim_played:
            self.uavs = {}
            self.torch_device = "cuda:0" if torch.cuda.is_available() else "cpu"

            try:
                self.rigid_prim_view = RigidPrimView(["/World/*/UAV_*",])
                self.rigid_prim_view.initialize()
            except Exception as e:
                carb.log_warn(f"[REMOTE COMMAND ext] Error initializing RigidPrimView: {e}")
                return

            self.init_uavs()
            self.uav_control = UAVcontrol(
                self.rigid_prim_view, 
                self.torch_device, 
                self.uavs, 
                carb.events.type_from_string(""), 
                self.event_stream)
            
            self.is_sim_played = True

    def init_uavs(self):
        pos, _ = self.rigid_prim_view.get_world_poses(indices=range(self.rigid_prim_view.count))

        for i in range(self.rigid_prim_view.count):
            uav_id = f"UAV_{i}"
            uav_state = "idle"
            uav_time = 0
            uav_pos = pos[i]
            uav_flightplan = None

            self.uavs[uav_id] = {
                "id": uav_id,
                "state": uav_state,
                "time": uav_time,
                "pos": uav_pos,
                "flightplan": uav_flightplan,
                "request": None
            }

    def build_ui(self):
        self._window = ui.Window("CG: NavSim - Command Generator", width=400, height=200)
        with self._window.frame:

            with ui.VStack(spacing=10, height=0):
                ui.Spacer(height=10)

                # UAV selector dropdown                    
                self.UAV_selector_dropdown = self.navsim_utils.build_uav_selector()

                with ui.HStack(spacing=10):


                    with ui.HStack(spacing=5):
                        ui.Button(
                            "ON", enable=False,
                            width=30,
                            style={
                                "background_color":ui.color.grey, 
                                "color":ui.color.white, 
                                "margin": 0})
                        self.rotors_CB = ui.CheckBox(tooltip="Rotors activation")
                        self.rotors_CB.model.set_value(True)

                    with ui.HStack():
                        ui.Button(
                            "duration", enable=False,
                            width=50,
                            style={
                                "background_color":ui.color.grey, 
                                "color":ui.color.white, 
                                "margin": 0})
                        self.duration_FF = ui.FloatField(tooltip="time executing this command")
                        self.duration_FF.model.set_value(1.0)
                        self.duration_FF.precision = 2

                with ui.HStack(spacing=10):

                    with ui.HStack():
                        ui.Button(
                            "velX", enable=False,
                            width=30,
                            style={
                                "background_color":ui.color.red, 
                                "color":ui.color.white, 
                                "margin": 0})
                        self.velX_FF = ui.FloatField(tooltip="velX parameter")
                        self.velX_FF.precision = 2


                    with ui.HStack():
                        ui.Button(
                            "velY", enable=False,
                            width=30,
                            style={
                                "background_color":ui.color.green, 
                                "color":ui.color.white, 
                                "margin": 0})
                        self.velY_FF = ui.FloatField(tooltip="velY parameter")
                        self.velY_FF.precision = 2

                    with ui.HStack():
                        ui.Button(
                            "velZ", enable=False,
                            width=30,
                            style={
                                "background_color":ui.color.blue, 
                                "color":ui.color.white, 
                                "margin": 0})
                        self.velZ_FF = ui.FloatField(tooltip="velZ parameter")
                        self.velZ_FF.precision = 2

                    with ui.HStack():
                        ui.Button(
                            "rotZ", enable=False,
                            width=20,
                            style={
                                "background_color":ui.color.orange, 
                                "color":ui.color.white, 
                                "margin": 0})
                        self.rotZ_FF = ui.FloatField(tooltip="rotZ parameter")
                        self.rotZ_FF.precision = 2

                with ui.HStack():
                    #send button
                    ui.Button(
                        "SEND", 
                        height=50,
                        clicked_fn = self.send_command)
                    
    def send_command(self):
        selected_uav = self.UAV_selector_dropdown.get_selection()
        if selected_uav is None:
            raise Exception("[REMOTE COMMAND ext] No drone selected")
                
        # Set command data structure
        command = Command(
            on = self.rotors_CB.checked, 
            velX = self.velX_FF.model.get_value_as_float(), 
            velY = self.velY_FF.model.get_value_as_float(), 
            velZ = self.velZ_FF.model.get_value_as_float(),
            rotZ = self.rotZ_FF.model.get_value_as_float(),
            duration = self.duration_FF.model.get_value_as_float()
        )

        uav_i = int(selected_uav.removeprefix("UAV_"))
        self.uav_control.commands[uav_i] = command
        self.uav_control.cmd_exp_time[uav_i] = self.current_time + command.duration

        # for i in range(self.rigid_prim_view.count):
        #     self.uav_control.commands[i] = command
        #     self.uav_control.cmd_exp_time[i] = self.current_time + command.duration

