# This software contains source code provided by NVIDIA Corporation.
# Copyright (c) 2022-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import omni.ui as ui

import omni.timeline

from omni.usd import StageEventType

from omni.isaac.ui.element_wrappers import CollapsableFrame, DropDown, CheckBox, TextBlock
from omni.isaac.ui.element_wrappers.core_connectors import LoadButton, ResetButton
from omni.isaac.ui.ui_utils import get_style
from omni.isaac.core.utils.stage import get_current_stage

from pxr import Sdf

from .scenes import Scenes

class UIBuilder:
    def __init__(self):
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []
        # UI elements created using a UIElementWrapper instance
        self.wrapped_ui_elements = []

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        # Run initialization for the provided example
        self._on_init()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar.
        This is called directly after build_ui().
        """
        pass

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """
        pass

    def on_physics_step(self, step: float):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing

        Args:
            step (float): Size of physics step
        """
        pass

    def on_stage_event(self, event):
        """Callback for Stage Events

        Args:
            event (omni.usd.StageEventType): Event Type
        """
        if event.type == int(StageEventType.OPENED):
            # If the user opens a new stage, the extension should completely reset
            self._reset_extension()

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from omni.isaac.ui.element_wrappers implement a cleanup function that should be called
        """
        for ui_elem in self.wrapped_ui_elements:
            ui_elem.cleanup()

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.
        This function will be called any time the UI window is closed and reopened.
        """
        world_controls_frame = CollapsableFrame("World Controls", collapsed=False)

        with world_controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                self._scene_selector_dropdown = DropDown(
                    "Select Scene", "SELECT", self._scene_list
                )
                self._scene_selector_dropdown.repopulate()
                self.wrapped_ui_elements.append(self._scene_selector_dropdown)

                self._load_btn = LoadButton(
                    "Load Button", "LOAD", setup_scene_fn=self._scenes.one_hundred_drones,
                    setup_post_load_fn=self._repop_drone_selector   # To tell the drone selector to start selecting
                )
                self._load_btn.set_world_settings(physics_dt=1 / 60.0, rendering_dt=1 / 60.0)
                self.wrapped_ui_elements.append(self._load_btn)

                self._reset_btn = ResetButton(
                    "Reset Button", "RESET", pre_reset_fn=None, post_reset_fn=self._on_post_reset_btn
                )
                self._reset_btn.enabled = False
                self.wrapped_ui_elements.append(self._reset_btn)


        # This is the Set Command frame, where the command is built
        set_command_frame = CollapsableFrame("Set Command", collapsed=False)
        
        with set_command_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                # Manipulable drone selector
                self._drone_selector_dropdown = DropDown("Select Drone", "SELECT", self._get_manipulable_UAVs)
                self._drone_selector_dropdown.enabled = False
                self.wrapped_ui_elements.append(self._drone_selector_dropdown)

                # Command parameters
                ui.Spacer(height=10)
                ui.Label("Command parameters:")

                # On
                self._on_checkbox = CheckBox("\tOn", default_value=False, 
                    tooltip="Tells the drone to execute a command")
                self.wrapped_ui_elements.append(self._on_checkbox)

                # Movement
                ui.Spacer(height=5)
                ui.Label("\tMovement:")

                # X velocity
                with ui.HStack():
                    ui.Label("\t\tX velocity")
                    self._x_velocity_slider = ui.FloatSlider(min=-1, max=1)
                
                # Y velocity
                with ui.HStack():
                    ui.Label("\t\tY velocity")
                    self._y_velocity_slider = ui.FloatSlider(min=-1, max=1)

                # Z velocity
                with ui.HStack():
                    ui.Label("\t\tZ velocity")
                    self._z_velocity_slider = ui.FloatSlider(min=-1, max=1)

                # Rotation
                ui.Spacer(height=5)
                ui.Label("\tRotation:")

                # Z rotation
                with ui.HStack():
                    ui.Label("\t\tZ rotation")
                    self._z_rotation_slider = ui.FloatSlider(min=-1, max=1)

                # Time duration
                ui.Spacer(height=5)
                with ui.HStack():
                    ui.Label("\tTime duration:")
                    self._time_duration_field = ui.IntField()

                # Send command
                ui.Spacer(height=15)
                self._send_command_button = ui.Button(text="SET COMMAND", clicked_fn=self._get_command_info)

                # Error message when no drone is selected
                self._command_fail_textBlock = TextBlock(label="ERROR:", text="There is no drone selected")
                self._command_fail_textBlock.visible = False

    ######################################################################################
    # Functions Below This Point Support The Provided Example And Can Be Deleted/Replaced
    ######################################################################################

    def _on_init(self):
        self._scenes = Scenes()

    def _scene_list(self):
        return ["100 Drones", "Example 2", "Example 3"]
    

    # -- FUNTION _get_manipulable_UAVs -------------------------------------------------------------------------------------
    # This function is used to get the drones that are manipulable, that is, able to receive and execute commands
    # ----------------------------------------------------------------------------------------------------------------------
    def _get_manipulable_UAVs(self):
        manipulable_UAVs = []
        stage = get_current_stage()

        # If UAVs are located within a parent prim as /World/abejorros
        if stage.GetPrimAtPath("/World/abejorros").IsValid():
            abejorros = stage.GetPrimAtPath("/World/abejorros")

            for abejorro in abejorros.GetChildren():
                manipulable_att = abejorro.GetAttribute("NavSim:Manipulable")

                if manipulable_att.Get():
                    manipulable_UAVs.append(abejorro.GetName())

        # If UAVs are NOT located within a specific parent prim
        else:
            world = stage.GetPrimAtPath("/World")
            
            for prim in world.GetChildren():
                manipulable_att = abejorro.GetAttribute("NavSim:Manipulable")

                if manipulable_att.Get():
                    manipulable_UAVs.append(prim.GetName())

        return manipulable_UAVs
    

    # -- FUNCTION _repop_drone_selector ------------------------------------------------------------------------------------
    # This method is called once the scene is loaded
    # It enables the drone dropdown selector and starts looking for manipulables drones
    # ----------------------------------------------------------------------------------------------------------------------
    def _repop_drone_selector(self):
        self._drone_selector_dropdown.enabled = True
        self._drone_selector_dropdown.repopulate()


    # -- FUNTION _get_command_info -----------------------------------------------------------------------------------------
    # This function is in charge of building the command message
    # It checks if a drone is selected and the search for the required parameters
    # If the 'On' parameter is set to False, it will return just {'On': False}
    # Otherwise, it will get the rest of the param
    # ----------------------------------------------------------------------------------------------------------------------
    def _get_command_info(self):
        if self._drone_selector_dropdown.get_selection() is not None:
            command_info = {}

            command_info["On"] = self._on_checkbox.get_value()
            
            if command_info["On"]:
                x_vel = round(self._x_velocity_slider.model.get_value_as_float(), 2)
                y_vel = round(self._y_velocity_slider.model.get_value_as_float(), 2)
                z_vel = round(self._z_velocity_slider.model.get_value_as_float(), 2)

                z_rot = round(self._z_rotation_slider.model.get_value_as_float(), 2)

                time = self._time_duration_field.model.get_value_as_int()

                command_info["vel"] = {"linear": {"x": x_vel, "y": y_vel, "z": z_vel}, "angular": {"z": z_rot}}
                command_info["duration"] = {"sec": time}

            # TEMPORAL

            # Call to set_command function from Iker
            manipulable_UAVs = self._get_manipulable_UAVs()
            prim_path = "/World/abejorros/" + manipulable_UAVs[self._drone_selector_dropdown.get_selection_index()]
            stage = get_current_stage()
            drone = stage.GetPrimAtPath(prim_path)

            # TEMPORAL

            # Debug
            print(command_info)

            # HIDE the error message when no drone is selected
            self._command_fail_textBlock.visible = False

        else:
            # SHOW error message when no drone is selected
            self._command_fail_textBlock.visible = True

    def _on_post_reset_btn(self):
        """
        This function is attached to the Reset Button as the post_reset_fn callback.
        The user may assume that their objects are properly initialized, and that the timeline is paused on timestep 0.

        They may also assume that objects that were added to the World.Scene have been moved to their default positions.
        I.e. the cube prim will move back to the position it was in when it was created in self._setup_scene().
        """

    def _update_scenario(self, step: float):
        """This function is attached to the Run Scenario StateButton.
        This function was passed in as the physics_callback_fn argument.
        This means that when the a_text "RUN" is pressed, a subscription is made to call this function on every physics step.
        When the b_text "STOP" is pressed, the physics callback is removed.

        Args:
            step (float): The dt of the current physics step
        """
        
    def _on_run_scenario_a_text(self):
        """
        This function is attached to the Run Scenario StateButton.
        This function was passed in as the on_a_click_fn argument.
        It is called when the StateButton is clicked while saying a_text "RUN".

        This function simply plays the timeline, which means that physics steps will start happening.  After the world is loaded or reset,
        the timeline is paused, which means that no physics steps will occur until the user makes it play either programmatically or
        through the left-hand UI toolbar.
        """
        self._timeline.play()

    def _on_run_scenario_b_text(self):
        """
        This function is attached to the Run Scenario StateButton.
        This function was passed in as the on_b_click_fn argument.
        It is called when the StateButton is clicked while saying a_text "STOP"

        Pausing the timeline on b_text is not strictly necessary for this example to run.
        Clicking "STOP" will cancel the physics subscription that updates the scenario, which means that
        the robot will stop getting new commands and the cube will stop updating without needing to
        pause at all.  The reason that the timeline is paused here is to prevent the robot being carried
        forward by momentum for a few frames after the physics subscription is canceled.  Pausing here makes
        this example prettier, but if curious, the user should observe what happens when this line is removed.
        """
        self._timeline.pause()

    def _reset_extension(self):
        """This is called when the user opens a new stage from self.on_stage_event().
        All state should be reset.
        """
        self._on_init()
        self._reset_ui()

    def _reset_ui(self):
        self._reset_btn.enabled = False
