import pickle
import base64
import numpy as np


import omni.kit.app
import omni.ext
from isaacsim.gui.components.ui_utils import ui
import carb.events
from isaacsim.gui.components.element_wrappers import *
import omni.timeline
import omni.physx
import omni.kit.window.file_importer


from uspace.flight_plan.flight_plan import FlightPlan
from navsim_utils.extensions_utils import ExtensionUtils
from navsim_utils.sim_utils import *


class FlightPlanGenerator(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.init_vars()
        self.build_ui()
                    
    def on_shutdown(self):
        self.on_physics_step_sub = None
        self.on_stop_sub = None

    def on_physics_step(self, step_size):
        self.current_time += step_size

    def on_timeline_stop(self, event):
        self.current_time = 0

    def on_timeline_play(self, event):
        if self.flightplan.waypoints:
            self.send_flightplan()

    def init_vars(self):
        self.extension_utils = ExtensionUtils()
        self.current_time = 0

        # Message bus event stream
        app_interface = omni.kit.app.get_app_interface()
        self.event_stream = app_interface.get_message_bus_event_stream()
        self.operator_uav_event = carb.events.type_from_string("NavSim.OperatorUAV")
        
        # Field variables
        self.position = [0, 0, 0]
        self.velocity = [0, 0, 0]
        self.time = 0
        self.fly_over = None

        # Drone selector handle
        self.UAV_selector_dropdown = None

        # UI field handles
        self.position_handles : list[ui.FloatDrag] = []
        self.velocity_handles : list[ui.FloatDrag] = []
        self.time_handle = None

        # Checkbox handles
        self.position_check_handle = None
        self.velocity_check_handle = None
        self.time_check_handle = None
        self.fly_over_check_handle = None

        # UI widgets
        self.waypoint_list_collapsable_frame = None
        self.waypoint_list_scrolling_frame = None
        self.waypoint_list = None
        self.empty_waypoint_list_label = None

        # Initialize flight plan
        self.flightplan = FlightPlan()

        # Callbacks
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
        self.on_play_sub = timeline_stream.create_subscription_to_pop_by_type(
            int(omni.timeline.TimelineEventType.PLAY),  
            self.on_timeline_play
        )
        
    def build_ui(self):
        # Create extension main window
        self.window = ui.Window(
            "FP: NavSim - Flight Plan Generator", 
            width=500, 
            height=800
        )
        self.window.deferred_dock_in("Layers")
        self.window.setPosition(25, 25)
        self.window.frame.set_style(self.extension_utils.Window_dark_style)

        # Populate window frame
        with self.window.frame:
            self.main_window_scrolling_frame = ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            )
            with self.main_window_scrolling_frame:
                with ui.VStack(
                    height=self.extension_utils.MINIMAL_HEIGHT, 
                    style=self.extension_utils.VStack_A, 
                    spacing=self.extension_utils.SPACING_S
                ):
                    ui.Spacer(height=self.extension_utils.MINIMAL_HEIGHT)
                    # Drone selector widget
                    self.UAV_selector_dropdown = self.extension_utils.build_uav_selector()
                    # Create transform frame
                    self.build_waypoint_frame()
                    # Create waypoint list
                    self.build_waypoint_list()
                    # Import/Export buttons
                    with ui.HStack(
                        spacing=self.extension_utils.SPACING_S, 
                        style=self.extension_utils.HStack_A
                    ):
                        # Import flight plan button
                        ui.Button(
                            "Import Flight Plan", 
                            height=50, 
                            clicked_fn=self.import_flightplan
                        )
                        # Export flight plan button
                        ui.Button(
                            "Export Flight Plan", 
                            height=50, 
                            clicked_fn=self.export_flightplan
                        )
                    # Send flight plan button
                    ui.Button(
                        "Send Flight Plan", 
                        height=50, 
                        clicked_fn=self.send_flightplan
                    )

    def build_waypoint_frame(self):
        with ui.CollapsableFrame(
            title="Waypoint", 
            style=self.extension_utils.CollapsableFrame_style
        ):
            with ui.VStack(
                spacing=self.extension_utils.SPACING_S, 
                style=self.extension_utils.VStack_B
            ):
                ui.Spacer(height=self.extension_utils.MINIMAL_HEIGHT)

                # Add time field
                with ui.HStack(spacing=self.extension_utils.SPACING_S):
                    # Time label
                    ui.Label(
                        "Time", 
                        style=self.extension_utils.Label_A, 
                        width=self.extension_utils.LABEL_PADDING
                    )
                    self.time_handle = ui.IntDrag(min=0, max=1000000, step=1)

                    # Time field checkbox
                    self.time_check_handle = ui.CheckBox(
                        width=self.extension_utils.MINIMAL_WIDTH
                    )
                    self.time_check_handle.model.set_value(True)

                # Position fields
                with ui.HStack(spacing=self.extension_utils.SPACING_S):
                    with ui.HStack(width=self.extension_utils.LABEL_PADDING):
                        ui.Label("Position", style=self.extension_utils.Label_A, width=50)
                        ui.Spacer()

                    # Axis fields
                    all_axis = ["X", "Y", "Z"]
                    colors = {"X": 0xFF5555AA, "Y": 0xFF76A371, "Z": 0xFFA07D4F}
                    for axis in all_axis:
                        with ui.HStack():
                            with ui.ZStack(width=15):

                                # Colored rectangles
                                ui.Rectangle(
                                    width=15, 
                                    height=20, 
                                    style={
                                        "background_color": colors[axis], 
                                        "border_radius": 3, 
                                        "corner_flag": ui.CornerFlag.LEFT
                                    }
                                )

                                # Axis letter label
                                ui.Label(
                                    axis, 
                                    style=self.extension_utils.Label_A, 
                                    alignment=ui.Alignment.CENTER
                                )

                            # FloatDrag widgets
                            self.position_handles.append(
                                ui.FloatDrag(min=-1000000, max=1000000, step=0.1)
                            )

                    # Position field checkboxes
                    self.position_check_handle = ui.CheckBox(
                        width=self.extension_utils.MINIMAL_WIDTH
                    )
                    self.position_check_handle.model.set_value(True)

                # Velocity fields
                with ui.HStack(spacing=self.extension_utils.SPACING_S):
                    with ui.HStack(width=self.extension_utils.LABEL_PADDING):
                        ui.Label("Velocity", style=self.extension_utils.Label_A, width=50)
                        ui.Spacer()

                    # Axis fields
                    all_axis = ["X", "Y", "Z"]
                    colors = {"X": 0xFF5555AA, "Y": 0xFF76A371, "Z": 0xFFA07D4F}
                    for axis in all_axis:
                        with ui.HStack():
                            with ui.ZStack(width=15):

                                # Colored rectangles
                                ui.Rectangle(
                                    width=15, 
                                    height=20, 
                                    style={
                                        "background_color": colors[axis], 
                                        "border_radius": 3, 
                                        "corner_flag": ui.CornerFlag.LEFT
                                    }
                                )

                                # Axis letter label
                                ui.Label(
                                    axis, 
                                    style=self.extension_utils.Label_A, 
                                    alignment=ui.Alignment.CENTER
                                )

                            # FloatDrag widgets
                            self.velocity_handles.append(
                                ui.FloatDrag(min=-1000000, max=1000000, step=0.1)
                            )

                    # Velocity field checkboxes
                    self.velocity_check_handle = ui.CheckBox(
                        width=self.extension_utils.MINIMAL_WIDTH
                    )
                    self.velocity_check_handle.model.set_value(True)

                # Fly over field
                with ui.HStack(spacing=self.extension_utils.SPACING_S):
                    # Fly over label
                    ui.Label(
                        "Fly over", 
                        style=self.extension_utils.Label_A, 
                        width=self.extension_utils.LABEL_PADDING
                    )

                    # Fly over field checkbox
                    self.fly_over_check_handle = ui.CheckBox(
                        width=self.extension_utils.MINIMAL_WIDTH
                    )
                    self.fly_over_check_handle.model.set_value(True)

                # Add waypoint id field
                with ui.HStack(spacing=self.extension_utils.SPACING_S):
                    # Waypoint id label
                    ui.Label(
                        "Waypoint ID", 
                        style=self.extension_utils.Label_A, 
                        width=self.extension_utils.LABEL_PADDING
                    )
                    self.waypoint_id = ui.StringField()

                    # Enabled field checkbox
                    self.waypoint_id_check_handle = ui.CheckBox(
                        width=self.extension_utils.MINIMAL_WIDTH
                    )
                    self.waypoint_id_check_handle.model.set_value(True)

                # Add buttons
                with ui.VStack(height=self.extension_utils.MINIMAL_HEIGHT):
                    # Add waypoint button
                    ui.Button(
                        "Add Waypoint", 
                        height=self.extension_utils.MINIMAL_HEIGHT, 
                        clicked_fn=self.add_waypoint
                    )

                    # Reset flight plan button
                    ui.Button(
                        "Reset Waypoints", 
                        height=self.extension_utils.MINIMAL_HEIGHT, 
                        clicked_fn=self.reset_waypoints
                    )

    def build_waypoint_list(self):
        self.waypoint_list_collapsable_frame = ui.CollapsableFrame(
            title="Waypoint list", 
            style=self.extension_utils.CollapsableFrame_style
        )
        with self.waypoint_list_collapsable_frame:
            with ui.VStack(
                spacing=self.extension_utils.SPACING_S, 
                style=self.extension_utils.VStack_B
            ):
                # Column labels
                with ui.HStack(style=self.extension_utils.HStack_A):
                    ui.Label("Label", alignment=ui.Alignment.LEFT)
                    ui.Label("Time", alignment=ui.Alignment.CENTER)
                    ui.Label("Position", alignment=ui.Alignment.LEFT)
                    ui.Label("Velocity", alignment=ui.Alignment.LEFT)
                    ui.Label("Fly over", alignment=ui.Alignment.LEFT)

                ui.Line()

                self.waypoint_list_scrolling_frame = ui.ScrollingFrame(
                    height=250,
                    horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                    style=self.extension_utils.ScrollingFrame_style
                )
                with self.waypoint_list_scrolling_frame:
                    self.waypoint_list = ui.VStack(
                        height=self.extension_utils.MINIMAL_HEIGHT, 
                        spacing=self.extension_utils.SPACING_S
                    )
                    self.reset_waypoints()

    def import_flightplan(self):
        """Import flight plan from a CSV file selected via file dialog"""
    
        def on_import_click(filename, dirname, selections):
            try:
                # Reset waypoints before importing
                self.reset_waypoints()

                # Load data from CSV file
                path = f"{dirname}/{filename}"
                data = np.loadtxt(path, delimiter=',', dtype=str)
                
                for row in data:
                    label = row[0]
                    time = float(row[1])
                    pos = np.array(row[2:5], dtype=float)
                    vel = np.array(row[5:8], dtype=float)
                    # heading = np.array(row[8:10], dtype=float)
                    heading = row[8:10]
                    
                    if heading[0] == 'None':
                        heading = None
                    else:
                        heading = np.array(heading, dtype=float)

                    # Add waypoint to the flight plan
                    self.flightplan.set_waypoint(
                        label=label, 
                        time=time, 
                        pos=pos, 
                        vel=vel, 
                        heading=heading
                    )
                
                self.flightplan.connect_waypoints()
                # Update UI to show imported waypoints
                self.print_waypoints()
                
            except Exception as e:
                print(f"[FP GENERATOR] Error importing flight plan: {str(e)}")
        
        # Create and show file import dialog
        import_dialog = omni.kit.window.file_importer.get_file_importer()
        import_dialog.show_window(
            title="Import Flight Plan",
            import_button_label="Import",
            import_handler=on_import_click,
            file_extension_types=[(".csv", "CSV Files")],
            file_filter_handler=None
        )

    def export_flightplan(self):
        """Export flight plan to a CSV file with name selected via file dialog"""
        
        if not self.flightplan.waypoints:
            print("[FP GENERATOR] No waypoints to export")
            return
        
        def on_export_click(filename, dirname, selections):
            if filename and dirname:
                # Ensure .csv extension
                if not filename.endswith('.csv'):
                    filename += '.csv'
                
                # Construct full path
                full_path = f"{dirname}/{filename}"
                
                try:
                    data = []

                    for wp in self.flightplan.waypoints:
                        label = wp.label
                        time = wp.t
                        pos = wp.pos
                        vel = wp.vel
                        heading = wp.heading if wp.heading is not None else [0, 0]
                        
                        data.append([label, time, *pos, *vel, *heading])
                    
                    # Save to CSV file
                    np.savetxt(full_path, np.array(data), delimiter=",", fmt="%s")
                    print(f"[FP GENERATOR] Successfully exported flight plan to: {full_path}")
                    
                except Exception as e:
                    print(f"[FP GENERATOR] Error exporting flight plan: {e}")
        
        # Create and show file export dialog
        export_dialog = omni.kit.window.file_importer.get_file_importer()
        export_dialog.show_window(
            title="Export Flight Plan",
            import_button_label="Export",
            import_handler=on_export_click,
            file_extension_types=[(".csv", "CSV Files")],
            file_filter_handler=None
        )

    def send_flightplan(self):
        selected_uav = self.UAV_selector_dropdown.get_selection()
        if selected_uav is None:
            raise Exception("[FP GENERATOR ext] No drone selected")

        serialized_fp = base64.b64encode(pickle.dumps(self.flightplan)).decode('utf-8')

        self.inform_operator(TypeMessage.CMD_FP_REQUEST, selected_uav, serialized_fp)

    def reset_waypoints(self):
        self.waypoint_list.clear()
        self.flightplan.waypoints.clear()
        with self.waypoint_list:
            ui.Spacer(height=self.extension_utils.MINIMAL_HEIGHT)
            self.empty_waypoint_list_label = ui.Label(
                "Waypoint list is empty", 
                style=self.extension_utils.Label_A, 
                alignment=ui.Alignment.CENTER_TOP
            )
                    
    def get_field_values(self):
        # Time
        if self.time_check_handle.model.get_value_as_bool():
            time = self.time_handle.model.get_value_as_int()
        else:
            time = None
        
        # Position
        if self.position_check_handle.model.get_value_as_bool():
            pos = [0,0,0]
            for i in range(3):
                pos[i] = self.position_handles[i].model.get_value_as_float()
        else:
            pos = None

        # Velocity
        if self.velocity_check_handle.model.get_value_as_bool():
            vel = [0,0,0]
            for i in range(3):
                vel[i] = self.velocity_handles[i].model.get_value_as_float()
        else:
            vel = None

        # Fly over
        fly_over = self.fly_over_check_handle.model.get_value_as_bool()

        # Waypoint ID
        if self.waypoint_id_check_handle.model.get_value_as_bool():
            label = self.waypoint_id.model.get_value_as_string()
        else:
            label = ""

        return time, pos, vel, fly_over, label

    def print_waypoints(self):
        self.waypoint_list.clear()
        
        with self.waypoint_list:                        
            for waypoint in self.flightplan.waypoints:
                with ui.HStack():
                    # Label
                    ui.Label(
                        waypoint.label, 
                        alignment=ui.Alignment.LEFT_CENTER, 
                        word_wrap=True
                    )
                    
                    # Time
                    ui.Label(f"{waypoint.t:.2f}", alignment=ui.Alignment.CENTER)

                    # Position data
                    with ui.VStack():
                        ui.Label(
                            f"{waypoint.pos[0]:.2f}", 
                            style={'color': self.extension_utils.colors["R"]}, 
                            alignment=ui.Alignment.LEFT
                        )
                        ui.Label(
                            f"{waypoint.pos[1]:.2f}", 
                            style={'color': self.extension_utils.colors["G"]}, 
                            alignment=ui.Alignment.LEFT
                        )
                        ui.Label(
                            f"{waypoint.pos[2]:.2f}", 
                            style={'color': self.extension_utils.colors["B"]}, 
                            alignment=ui.Alignment.LEFT
                        )

                    # Velocity data
                    with ui.VStack():
                        ui.Label(
                            f"{waypoint.vel[0]:.2f}", 
                            style={'color': self.extension_utils.colors["R"]}, 
                            alignment=ui.Alignment.LEFT
                        )
                        ui.Label(
                            f"{waypoint.vel[1]:.2f}", 
                            style={'color': self.extension_utils.colors["G"]}, 
                            alignment=ui.Alignment.LEFT
                        )
                        ui.Label(
                            f"{waypoint.vel[2]:.2f}", 
                            style={'color': self.extension_utils.colors["B"]}, 
                            alignment=ui.Alignment.LEFT
                        )

                    # Fly over
                    if waypoint.fly_over:
                        ui.Label(
                            str(waypoint.fly_over), 
                            style={'color' : self.extension_utils.colors["G"]}, 
                            alignment=ui.Alignment.CENTER
                        )
                    else:
                        ui.Label(
                            str(waypoint.fly_over), 
                            style={'color' : self.extension_utils.colors["R"]}, 
                            alignment=ui.Alignment.CENTER
                        )

                ui.Separator()

    def add_waypoint(self):
        # Update variables from UI fields
        time, pos, vel, fly_over, label = self.get_field_values()

        # Add waypoint to the list
        self.flightplan.set_waypoint(label=label, time=time, pos=pos, vel=vel)
        self.flightplan.connect_waypoints()
        
        self.print_waypoints()

    def inform_operator(self, type_message, uav_id, fp):
        """Inform the operator about the flightplan to be sent to the UAV"""
        
        payload = {"type_message": type_message}
        msg = {"sender": TypeSender.FLIGHTPLAN_GENERATOR}

        match type_message:
            case TypeMessage.CMD_FP_REQUEST:
                msg["request"] = {
                    "uav_id": uav_id,
                    "fp": fp
                }

        payload["msg"] = msg

        self.event_stream.push(self.operator_uav_event, payload=payload)
