# Standard library imports
import sys, os


# Related third party imports
import omni.ext
import omni.ui as ui
import omni.kit.viewport.window
from omni.kit.widget.viewport import ViewportWidget

project_root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
# Add root 'ov' folder to sys.path
if project_root_path not in sys.path:
    sys.path.append(project_root_path)


KIT_GREEN = 0xFF8A8777
LABEL_PADDING = 120


Window_dark_style = {
    "Window": {"background_color": 0xFF444444}
}


VStack_A = {
    "VStack": {
        "margin_width": 10, 
        "margin_height": 0
    }
}


VStack_B = {
    "VStack": {"margin_width": 15}
}


Label_A = {
    "Label": {
        "font_size": 12,
        "color": 0xFFDDDDDD
    }
}


colors = {
    "R": 0xFF5555AA,
    "G": 0xFF76A371,
    "B": 0xFFA07D4F
}


CollapsableFrame_style = {
    "CollapsableFrame": {
        "background_color": 0xFF343432,
        "secondary_color": 0xFF343432,
        "color": 0xFFAAAAAA,
        "border_radius": 4.0,
        "border_color": 0x0,
        "border_width": 0,
        "font_size": 14,
        "padding": 0,
    },
    "HStack::header": {"margin": 5},
    "CollapsableFrame:hovered": {"secondary_color": 0xFF3A3A3A},
    "CollapsableFrame:pressed": {"secondary_color": 0xFF343432},
}

ScrollingFrame_style = {
    "ScrollingFrame": {
        "background_color": 0xFF343432,
        "secondary_color": 0xFF343432,
        "color": 0xFFAAAAAA,
        "border_radius": 4.0,
        "border_color": 0x0,
        "border_width": 0,
        "font_size": 14,
        "padding": 0,
    },
    "HStack::header": {"margin": 5},
    "ScrollingFrame:hovered": {"secondary_color": 0xFF3A3A3A},
    "ScrollingFrame:pressed": {"secondary_color": 0xFF343432},
}

class Uav_raceExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        self.create_vars()
        self.build_ui()

    def on_shutdown(self):
        pass

    def create_vars(self):
        self.perspective_camera_path = "/OmniverseKit_Persp"
        self.follow_UAV_camera_path = "/manual_controller_CAM"
        self.amount_users = 4
        self.viewports = {}

    def build_ui(self):
        self.window = ui.Window("NavSim - UAV Race", width=200, height=200)
        self.window.frame.set_style(Window_dark_style)

        with self.window.frame:
            with ui.VStack(style=VStack_A, height=0):
                self.add_viewports_button = ui.Button("ADD VIEWPORT", clicked_fn=self.add_viewports)

                # self.add_viewports_collapsable = ui.CollapsableFrame("Joysticks")
                # self.add_viewports_collapsable.set_style(CollapsableFrame_style)
                # with self.add_viewports_collapsable:
                #     ui.Label("Joystick1")

    # def add_viewports(self):
    #     amount_viewports = len(self.viewports)
        
    #     viewport_width = ui.Workspace.get_main_window_width()/4
    #     viewport_height = ui.Workspace.get_main_window_height()/3

    #     viewport_window = ui.Window(title=f"User {amount_viewports + 1}", width=viewport_width, height=viewport_height+20, 
    #                                 raster_policy=ui.RasterPolicy.NEVER)
    #     viewport_window.set_visibility_changed_fn(self.viewport_on_visibility_change)
    #     with viewport_window.frame:
    #         viewport_widget = ViewportWidget(resolution=(640, 360))

    #     viewport_api = viewport_widget.viewport_api
    #     viewport_api.camera_path = self.follow_UAV_camera_path

    #     self.viewports[viewport_window.title] = (viewport_window, viewport_widget)

    def add_viewports(self):
        amount_viewports = 0
        for viewport_window in omni.kit.viewport.window.get_viewport_window_instances():
            if not viewport_window.visible:
                viewport_window.viewport_widget.destroy()
                viewport_window.destroy()
                continue

            amount_viewports += 1
        
        viewport_width = ui.Workspace.get_main_window_width()/4
        viewport_height = ui.Workspace.get_main_window_height()/3
        render_resolution_scale = 0.5

        viewport_window = omni.kit.viewport.window.ViewportWindow(name=f"User {amount_viewports}", 
                                                                  width=viewport_width, height=viewport_height)

        viewport_window.setPosition(viewport_width*(amount_viewports - 1), viewport_height*2)
        viewport_window.viewport_api.set_active_camera(self.follow_UAV_camera_path)
        viewport_window.set_visibility_changed_fn(self.viewport_on_visibility_change)

        viewport_api = viewport_window.viewport_api

    def viewport_on_visibility_change(self, visible):
        for viewport_window in omni.kit.viewport.window.get_viewport_window_instances():
            if not viewport_window.visible:
                # self.viewports.pop(viewport_window.title)
                viewport_window.viewport_widget.destroy()
                viewport_window.destroy()

    # def viewport_on_visibility_change(self, visible):
    #     for viewport_window in self.viewports.values():
    #         if not viewport_window.visible:
    #             viewport_window.destroy()
    #             self.viewports.pop(viewport_window.title)