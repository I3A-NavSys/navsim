import matplotlib.pyplot as plt

from isaacsim.gui.components.ui_utils import ui

from navsim_utils.extensions_utils import ExtensionUtils
from uspace.flight_plan.flight_plan import FlightPlan

class UIBuilder:
    def __init__(self, navsim_manager):
        self.extension_utils = ExtensionUtils()
        self.navsim_manager = navsim_manager
        self.current_flightplans = None

    def build_ui(self):
        self.window = ui.Window("UAV OP", width=300, height=300)
        self.window.frame.set_style(self.extension_utils.Window_dark_style)

        with self.window.frame:
            with ui.ScrollingFrame(
                horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
            ):
                with ui.VStack(height=0, spacing=self.extension_utils.SPACING_S):
                    ui.Button(
                        text="Mostrar Flightplan",
                        clicked_fn=self.show_flightplan,
                    )

    
    # ------------------
    # -- UI Callbacks --
    # ------------------
    def show_flightplan(self):
        # for uav_operator_id, uav_id, flightplan in self.current_flightplans:
        #     if uav_operator_id == "UAV_OP_01" and uav_id == "UAV_03":
                # if not plt.fignum_exists("UAV_OP_01-UAV_03"):
                #     flightplan.position_figure("UAV_OP_01-UAV_03", 0.1)

        fp = FlightPlan()

        fp.set_waypoint(
            time=0,
            pos=[0, 0, 0],
            vel=[10, 0, 0]
        )

        fp.set_waypoint(
            time=10,
            pos=[100, 0, 0],
            vel=[0, 10, 0]
        )

        fp.set_waypoint(
            time=20,
            pos=[100, 100, 0],
            vel=[0, 0, 0]
        )

        fp.connect_waypoints()

        fp.position_figure("POS", 0.1)
        fp.velocity_figure("VEL", 0.1)
        fp.acceleration_figure("ACC", 0.1)


