import pickle   # Serialization
import base64   # Parsing to string


import omni.kit.app
import omni.ext
from isaacsim.gui.components.ui_utils import ui
from isaacsim.gui.components.element_wrappers import *
import carb.events
import omni.timeline
import omni.physx


from uspace.flight_plan.command import Command
from navsim_utils.extensions_utils import ExtensionUtils


class TypeMessage:
    EXTENSSION_ON_OFF = "extension_on_off"
    CMD_FP_REQUEST = "cmd_fp_request"
    USPACE = "uspace"

class AerialOperation:
    COMMAND = "command"
    FLIGHTPLAN = "flightplan"

class CommandGenerator(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.init_vars()
        self.build_ui()

    def on_shutdown(self):
        self.on_physics_step_sub = None
        self.on_stop_sub = None

    def on_physics_step(self, step_size:int):
        self.current_time += step_size

    def on_stop(self, event):
        self.current_time = 0

    def init_vars(self):        
        self.navsim_utils = ExtensionUtils()
        self.current_time = 0

        self.physx_interface = omni.physx.get_physx_interface()
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_step_events(
            self.on_physics_step
        )

        self.timeline = omni.timeline.get_timeline_interface()
        timeline_stream = self.timeline.get_timeline_event_stream()
        self.on_stop_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), 
            self.on_stop
        )

        self.event_stream = omni.kit.app.get_app_interface().get_message_bus_event_stream()
        self.operator_event = carb.events.type_from_string("NavSim.Operator")

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
                                "margin": 0
                            }
                        )
                        self.rotors_CB = ui.CheckBox(tooltip="Rotors activation")
                        self.rotors_CB.model.set_value(True)

                    with ui.HStack():
                        ui.Button(
                            "duration", enable=False,
                            width=50,
                            style={
                                "background_color":ui.color.grey, 
                                "color":ui.color.white, 
                                "margin": 0
                            }
                        )
                        self.duration_FF = ui.FloatField(
                            tooltip="time executing this command"
                        )
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
                                "margin": 0
                            }
                        )
                        self.velX_FF = ui.FloatField(tooltip="velX parameter")
                        self.velX_FF.precision = 2


                    with ui.HStack():
                        ui.Button(
                            "velY", enable=False,
                            width=30,
                            style={
                                "background_color":ui.color.green, 
                                "color":ui.color.white, 
                                "margin": 0
                            }
                        )
                        self.velY_FF = ui.FloatField(tooltip="velY parameter")
                        self.velY_FF.precision = 2

                    with ui.HStack():
                        ui.Button(
                            "velZ", enable=False,
                            width=30,
                            style={
                                "background_color":ui.color.blue, 
                                "color":ui.color.white, 
                                "margin": 0
                            }
                        )
                        self.velZ_FF = ui.FloatField(tooltip="velZ parameter")
                        self.velZ_FF.precision = 2

                    with ui.HStack():
                        ui.Button(
                            "rotZ", enable=False,
                            width=20,
                            style={
                                "background_color":ui.color.orange, 
                                "color":ui.color.white, 
                                "margin": 0
                            }
                        )
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
        serialized_cmd = base64.b64encode(pickle.dumps(command)).decode('utf-8')

        self.inform_operator(TypeMessage.CMD_FP_REQUEST, uav_i, serialized_cmd)

    def inform_operator(self, type_message, uav_id, cmd):
        """Inform the operator about the command to be sent to the UAV"""
        
        payload = {"type_message": type_message}

        match type_message:
            case TypeMessage.CMD_FP_REQUEST:
                request = {
                    "uav_id": uav_id,
                    "cmd": cmd
                }

                payload["operation"] = AerialOperation.COMMAND
                payload["request"] = request

        self.event_stream.push(self.operator_event, payload=payload)
