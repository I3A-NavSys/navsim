from isaacsim.gui.components import ui
from omni.ui import color


from navsim_utils.sim_utils import UAVStatus
from navsim_utils.extensions_utils import MinimalComboBoxModel


class UIBuilder:
    def __init__(self, extension_utils, operators):
        # Utils
        self.extension_utils = extension_utils
        self.operators = operators

    def build_uav_info(self) -> None:
        def toggle_flightplan_debug(
            selected_operator: dict[str, dict], 
            selected_uav: dict[str, any]
        ) -> None:
            # Inverted logic for checkbox as it does not update value until after click event
            checked = not self.ui_show_flightplan_checkbox.model.get_value_as_bool()
            
            # Show flightplan only if UAV is busy
            if checked:
                if selected_uav["status"] != UAVStatus.BUSY:
                    self.logger.warning(f"{uav_id} is not busy. Cannot show flightplan.")
                    return
                
                mission_id = selected_uav["mission"]
                mission = selected_operator["missions"][mission_id]
                flightplan = mission["flightplan"]
                
            # Hide flightplan
            else:
                pass
        def toggle_uav_flight_state_debug(
            selected_operator: dict[str, dict], 
            selected_uav: dict[str, any]
        ) -> None:
            # Inverted logic for checkbox as it does not update value until after click event
            checked = not self.ui_show_uav_flight_state_checkbox.model.get_value_as_bool()
            
            # Show UAV flight state only if UAV is busy
            if checked:
                if selected_uav["status"] != UAVStatus.BUSY:
                    self.logger.warning(f"{uav_id} is not busy. Cannot show UAV flight state.")
                    return
                
            # Hide UAV flight state
            else:
                pass
        
        def build_location(location: list[float]) -> None:
                with ui.ZStack():
                    # Container
                    with ui.HStack():
                        # Padding left
                        ui.Frame(width=ui.Percent(20))
                        
                        ui.Rectangle(
                            width=ui.Percent(60),
                            # height=150,
                            style={
                                "background_color": color("#5B5B5B"),
                                "border_radius": 5,
                            }
                        )
                    
                    # Location
                    with ui.VStack(height=0):
                        # Title
                        ui.Spacer(height=1)
                        ui.Label(
                            "Location", 
                            alignment=ui.Alignment.CENTER_TOP,
                            style={"font_size": 18}
                        )
                        
                        
                        with ui.ZStack():
                            # Container
                            with ui.HStack():
                                # Padding left
                                ui.Frame(width=ui.Percent(25))
                                        
                                ui.Rectangle(
                                    width=ui.Percent(50),
                                    height=90,
                                    style={
                                        "background_color": color("#3A3A3A"),
                                        "border_radius": 5,
                                        "margin": 5,
                                    }
                                )
                                
                            # Content
                            with ui.HStack():
                                # Padding left
                                ui.Frame(width=ui.Percent(27))
                                
                                with ui.VGrid(column_count=2):
                                    ui.Label(
                                        "X",
                                        style={"color": color("#FF3838")}
                                    )
                                    ui.Label(
                                        str(round(location[0], 2)),
                                        alignment=ui.Alignment.RIGHT,
                                        style={"color": color("#FF3838")}
                                    )
                                    
                                    ui.Label(
                                        "Y",
                                        style={"color": color("#77FF38")}
                                    )
                                    ui.Label(
                                        str(round(location[1], 2)),
                                        alignment=ui.Alignment.RIGHT,
                                        style={"color": color("#77FF38")}
                                    )
                                    
                                    ui.Label(
                                        "Z",
                                        style={"color": color("#38A9FF")}
                                    )
                                    ui.Label(
                                        str(round(location[2], 2)),
                                        alignment=ui.Alignment.RIGHT,
                                        style={"color": color("#38A9FF")}
                                    )
                                    
                                # Padding right
                                ui.Frame(width=ui.Percent(27))   
        def build_parameters(selected_uav: dict[str, any]) -> None:
            with ui.ZStack():
                # Container
                with ui.HStack():
                    # Padding left
                    ui.Frame(width=ui.Percent(5))
                    
                    ui.Rectangle(
                        width=ui.Percent(90),
                        # height=205,
                        style={
                            "background_color": color("#5B5B5B"),
                            "border_radius": 5,
                        }
                    )
                
                # Parameters
                with ui.VStack(height=0):
                    # Title
                    ui.Spacer(height=1)
                    ui.Label(
                        "Parameters", 
                        alignment=ui.Alignment.CENTER_TOP,
                        style={"font_size": 18}
                    )
                    
                    # Content
                    with ui.ZStack():
                        # Container
                        with ui.HStack():
                            # Padding left
                            ui.Frame(width=ui.Percent(10))
                                    
                            ui.Rectangle(
                                width=ui.Percent(80),
                                height=145,
                                style={
                                    "background_color": color("#3A3A3A"),
                                    "border_radius": 5,
                                }
                            )
                        
                        with ui.HStack():
                            # Padding left
                            ui.Frame(width=ui.Percent(12))
                            
                            with ui.VGrid(column_count=2):
                                ui.Label("Type:")
                                ui.Label(
                                    selected_uav["type"],
                                    alignment=ui.Alignment.RIGHT
                                )
                                
                                ui.Label("Status:")
                                ui.Label(
                                    selected_uav["status"],
                                    alignment=ui.Alignment.RIGHT
                                )
                                
                                ui.Label("Mission:")
                                ui.Label(
                                    selected_uav["mission"],
                                    alignment=ui.Alignment.RIGHT
                                )
                                
                                ui.Label("Time:")
                                ui.Label(
                                    selected_uav["time"],
                                    alignment=ui.Alignment.RIGHT
                                )
                                
                                ui.Label("Battery:")
                                ui.Label(
                                    str(selected_uav["battery"]) + " %",
                                    alignment=ui.Alignment.RIGHT
                                )
                                
                            # Padding right
                            ui.Frame(width=ui.Percent(12))
        def build_debug_buttons(
            selected_operator: dict[str, dict], 
            selected_uav: dict[str, any]
        ) -> None:
            with ui.ZStack():
                # Container
                with ui.HStack():
                    # Padding left
                    ui.Frame(width=ui.Percent(18.5))
                    
                    ui.Rectangle(
                        width=ui.Percent(65),
                        # height=205,
                        style={
                            "background_color": color("#5B5B5B"),
                            "border_radius": 5,
                        }
                    )
                    
                # Buttons
                with ui.VStack(height=0):
                    # Title
                    ui.Spacer(height=1)
                    ui.Label(
                        "Debug Options", 
                        alignment=ui.Alignment.CENTER_TOP,
                        style={"font_size": 18}
                    )
                
                    # Show Flightplan
                    with ui.ZStack():
                        # Container
                        with ui.HStack():
                            # Padding left
                            ui.Frame(width=ui.Percent(20))
                            
                            ui.Rectangle(
                                width=ui.Percent(62.5),
                                height=40,
                                style={
                                    "background_color": color("#3A3A3A"),
                                    "border_radius": 10,
                                }
                            )
                        
                        # Checkbox
                        with ui.HStack():
                            # Padding left
                            ui.Frame(width=ui.Percent(25))
                            
                            self.ui_show_flightplan_checkbox = ui.CheckBox(
                                width=0,
                                style={"margin_height": 12},
                                mouse_pressed_fn=lambda x, y, b, m: 
                                    toggle_flightplan_debug(
                                        selected_operator, 
                                        selected_uav
                                    ),
                            )
                            
                            ui.Label(
                                "Show Flightplan", 
                                alignment=ui.Alignment.CENTER,
                                style={"font_size": 16}
                            )
                            
                            # Padding right
                            ui.Frame(width=ui.Percent(25))
                        
                    # Show UAV Flight State
                    with ui.ZStack():
                        # Container
                        with ui.HStack():
                            # Padding left
                            ui.Frame(width=ui.Percent(20))
                            
                            ui.Rectangle(
                                width=ui.Percent(62.5),
                                height=40,
                                style={
                                    "background_color": color("#3A3A3A"),
                                    "border_radius": 10,
                                }
                            )
                        
                        # Checkbox
                        with ui.HStack():
                            # Padding left
                            ui.Frame(width=ui.Percent(25))
                            
                            self.ui_show_uav_flight_state_checkbox = ui.CheckBox(
                                width=0,
                                style={"margin_height": 12},
                                mouse_pressed_fn=lambda x, y, b, m: 
                                    toggle_uav_flight_state_debug(
                                        selected_operator, 
                                        selected_uav
                                    ),
                            )
                            
                            ui.Label(
                                "Show UAV Flight State", 
                                alignment=ui.Alignment.CENTER,
                                style={"font_size": 16}
                            )
                            
                            # Padding right
                            ui.Frame(width=ui.Percent(25))
                        
                        
        self.ui_content_frame.clear()
        
        # Get selected operator
        operator_id = self.ui_operator_dropdown.model.get_selection()
        selected_operator = self.operators[operator_id]
        
        # Check if it has UAVs
        uavs_ids = list(selected_operator["uavs"].keys())
        if not uavs_ids:    return
        
        # Update info dropdown with UAV ids
        self.ui_info_dropdown.model.set_children(uavs_ids)
        
        # Get selected UAV
        uav_id = self.ui_info_dropdown.model.get_selection()
        selected_uav = selected_operator["uavs"][uav_id]
        
        
        # Update content frame with selected UAV info
        with self.ui_content_frame:
            with ui.VStack(height=0):
                build_location(selected_uav["location"])
                build_parameters(selected_uav)
                build_debug_buttons(selected_operator, selected_uav)

    def build_mission_info(self) -> None:
        pass
    
    def build_statistics_info(self) -> None:
        pass
    
    
    def build_title(self) -> None:
        ui.Spacer(height=10)
        
        with ui.ZStack():
            ui.Rectangle(
                height=60,
                style={
                    "background_color": color("#353535"),
                    "border_radius": 10,
                }
            )
            ui.Label(
                "NavSim - UAV Operator", 
                alignment=ui.Alignment.CENTER,
                style={"font_size": 20}
            )
            
        ui.Spacer(height=10)
        
    def build_operator_dropdown(self) -> None:
        ui.Label("Select UAV Operator:")
        self.ui_operator_dropdown = ui.ComboBox(MinimalComboBoxModel())
        self.ui_operator_dropdown.model.set_children(
            list(self.operators.keys())
        )
                    
    def build_information_buttons(self) -> None:
        with ui.ZStack(style={"margin": 5}):
            ui.Rectangle(
                style={
                    "background_color": color("#5A5A5A"),
                    "border_radius": 5,
                }
            )
            
            with ui.VStack(height=0):
                with ui.HStack(spacing=self.extension_utils.SPACING_S):
                    ui.Button(
                        text="UAVs",
                        height=40,
                        clicked_fn=self.build_uav_info,
                    )
                    
                    ui.Button(
                        text="Missions",
                        height=40,
                        clicked_fn=self.build_mission_info,
                    )
                    
                    ui.Button(
                        text="Statistics",
                        height=40,
                        clicked_fn=self.build_statistics_info,
                    )
                    
                self.ui_info_dropdown = ui.ComboBox(MinimalComboBoxModel())
                self.ui_info_dropdown.model.add_item_changed_fn(
                    lambda m, i: print("yeah!")
                )
            
    def build_content_frame(self) -> None:
        with ui.ZStack(height=420):
            with ui.HStack():
                # Padding left
                ui.Frame(width=ui.Percent(7.5))
                
                ui.Rectangle(
                    width=ui.Percent(85),
                    style={
                    "background_color": color("#757575"),
                    "border_radius": 5,
                    }
                )
            self.ui_content_frame = ui.VStack(height=0, style={"margin": 5})
            
            
    def build_ui(self) -> None:
        self.window = ui.Window("UAV OP", width=300, height=300)
        self.window.frame.set_style(self.extension_utils.Window_dark_style)

        with self.window.frame:
            with ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            ):
                with ui.VStack(height=0, spacing=self.extension_utils.SPACING_S):
                    self.build_title()
                    self.build_operator_dropdown()
                    
                    # Separator
                    ui.Rectangle(height=2, style={"background_color": color("#CCCCCC")})
                    
                    self.build_information_buttons()
                    self.build_content_frame()


