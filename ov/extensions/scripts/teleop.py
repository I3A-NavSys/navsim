import omni.appwindow
import carb.input

from carb.input import KeyboardEventType, KeyboardEvent
from omni.kit.scripting import BehaviorScript
from pxr import Sdf

class UamMinidrone(BehaviorScript):
    def on_init(self):
        #Get Keyboard
        app_window = omni.appwindow.get_default_app_window()
        self.keyboard = app_window.get_keyboard()
        input = carb.input.acquire_input_interface()
        self.keyboard_sub_id = input.subscribe_to_keyboard_events(self.keyboard, self.on_keyboard_input)

        # Mass
        self.mass_attr = self.prim.GetAttribute("physics:mass")

        # Force
        self.force_atr = self.prim.CreateAttribute("physxForce:force", Sdf.ValueTypeNames.Float3)
        self.force_atr = self.prim.GetAttribute("physxForce:force")

        # Torque
        self.torque_atr = self.prim.CreateAttribute("physxForce:torque", Sdf.ValueTypeNames.Float3)
        self.torque_atr = self.prim.GetAttribute("physxForce:torque")

        # Apply gravity force and initialize linear force
        self.gravity_force = self.mass_attr.Get() * 9.81
        self.force_atr.Set((0, 0, self.gravity_force))
        self.linear_force = 0

    def on_destroy(self):
        pass

    def on_play(self):
        pass

    def on_pause(self):
        pass

    def on_stop(self):
        pass

    def on_update(self, current_time: float, delta_time: float):
        self.force_atr.Set((0, 0, self.gravity_force + self.linear_force))

    def on_keyboard_input(self, e: carb.input.KeyboardEvent):
        # Increase upward force
        if e.input == carb.input.KeyboardInput.W:
            if e.type == KeyboardEventType.KEY_PRESS or e.type == KeyboardEventType.KEY_REPEAT:
                if self.linear_force <= 1:
                    self.linear_force += 0.1

        # Decrease upward force
        elif e.input == carb.input.KeyboardInput.S:
            if e.type == KeyboardEventType.KEY_PRESS or e.type == KeyboardEventType.KEY_REPEAT:
                if self.linear_force > 0:
                    self.linear_force -= 0.1