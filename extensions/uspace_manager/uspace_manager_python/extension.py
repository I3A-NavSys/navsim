import omni.ext
from isaacsim.gui.components.ui_utils import ui
import carb.events
import omni.timeline
import omni.physx
import omni.kit.app


import pickle
import base64


from navsim_utils.sim_utils import *
from navsim_utils.extensions_utils import ExtensionUtils
from uspace.grid_planner.grid_planner import GridPlanner


class USpaceClients(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.init_vars()
        self.build_ui()
        
    def on_shutdown(self):
        self.on_physics_step_sub = None
        self.on_stop_sub = None
        self.event_sub = None
        
    def on_physics_step(self, step_size:int):
        self.current_time += step_size

    def on_timeline_stop(self, event):
        self.current_time = 0
    
    def init_vars(self):
        self.physx_interface = omni.physx.get_physx_interface()
        self.on_physics_step_sub = self.physx_interface.subscribe_physics_step_events(
            self.on_physics_step
        )

        self.timeline = omni.timeline.get_timeline_interface()
        timeline_stream = self.timeline.get_timeline_event_stream()
        self.on_stop_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.STOP), 
            self.on_timeline_stop
        )
        
        self.event_stream = omni.kit.app.get_app().get_message_bus_event_stream()
        self.operator_event = carb.events.type_from_string("NavSim.Operator")
        self.uspace_manager_event = carb.events.type_from_string("NavSim.USpaceManager")
        self.event_sub = self.event_stream.create_subscription_to_push_by_type(
            self.uspace_manager_event, 
            self.event_listener
        )

        self.current_time = 0
        self.time_manager = TimeManager()
        self.geospatial_manager = GeospatialManager()
        self.extension_utils = ExtensionUtils()
        self.gp = GridPlanner()

    def event_listener(self, event):
        payload = event.payload

        match payload["type_message"]:
            case TypeMessage.USPACE:
                self.handle_uspace_msg(payload)

    def handle_uspace_msg(self, payload):
        msg = payload["msg"]

        match msg["sender"]:
            case TypeSender.OPERATOR_UAV:
                self.handle_operator_uav_msg(msg)

    def handle_operator_uav_msg(self, msg):
        pass

    def build_ui(self):        
        self.window = ui.Window(
            "USM: NavSim - Uspace manager", 
            width=300, 
            height=300,
        )

        with self.window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=self.extension_utils.SPACING_S, height=0):
                    self.build_ui_title()
                    self.build_ui_grid_parameters()

    def build_ui_title(self):
        ui.Spacer(height=10)
        ui.Label(
            "NAVSIM - USPACE MANAGER", 
            alignment=ui.Alignment.CENTER, 
            style={"font_size": 20, "font_weight": "bold"}
        )
        ui.Spacer(height=5)

    def build_ui_grid_parameters(self):
        with ui.CollapsableFrame(
            "Grid Parameters", 
            collapsed=False,
            style=self.extension_utils.CollapsableFrame_style,
        ):
            # Content
            with ui.VStack():
                ui.Spacer(height=10)

                with ui.ZStack(
                    style={"margin_width": 5},
                ):
                    # Background
                    ui.Rectangle(
                        style={
                            "background_color": 0xFF5b5b5b, 
                            "border_radius": 5, 
                            "corner_flag": ui.CornerFlag.ALL
                        }
                    )
                    
                    # Parameters
                    with ui.VStack(
                        spacing=self.extension_utils.SPACING_S, 
                        height=0, 
                        style=self.extension_utils.VStack_A,
                    ):
                        ui.Spacer(height=5)

                        with ui.HStack():
                            ui.Label("Cell size")
                            self.ui_grid_cell_size = ui.IntField()
                            self.ui_grid_cell_size.model.set_value(100)
                        with ui.HStack():
                            ui.Label("Slot time")
                            self.ui_grid_slot_time = ui.IntField()
                            self.ui_grid_slot_time.model.set_value(10)
                        with ui.HStack():
                            ui.Label("X level height")
                            self.ui_grid_x_level_height = ui.IntField()
                            self.ui_grid_x_level_height.model.set_value(60)
                        with ui.HStack():
                            ui.Label("Y level height")
                            self.ui_grid_y_level_height = ui.IntField()
                            self.ui_grid_y_level_height.model.set_value(100)

                        self.ui_grid_set_params = ui.Button(
                            "SET PARAMETERS", 
                            height=50, 
                            clicked_fn=self.set_grid_parameters
                        )

                        ui.Spacer(height=5)

                ui.Spacer(height=10)

    def set_grid_parameters(self):
        self.gp.cell_side = self.ui_grid_cell_size.model.get_value_as_int()
        self.gp.slot_time = self.ui_grid_slot_time.model.get_value_as_int()
        self.gp.x_height = self.ui_grid_x_level_height.model.get_value_as_int()
        self.gp.y_height = self.ui_grid_y_level_height.model.get_value_as_int()

    def inform_operator(self, type_message, request=None):
        payload = {"type_message": type_message}
        msg = {"sender": TypeSender.USPACE_MANAGER}

        match type_message:
            case TypeMessage.USPACE:
                msg["request"] = request

        payload["msg"] = msg

        self.event_stream.push(self.operator_event, payload=payload)