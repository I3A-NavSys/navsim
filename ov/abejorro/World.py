import carb
from omni.kit.scripting import BehaviorScript

import omni.appwindow
import carb.input
from pxr import Gf



class World(BehaviorScript):
    def on_init(self):
        carb.log_info(f"{type(self).__name__}.on_init()->{self.prim_path}")

       


    def on_destroy(self):
        carb.log_info(f"{type(self).__name__}.on_destroy()->{self.prim_path}")

        self.keyboard_sub_id = None




    def on_play(self):
        carb.log_info(f"{type(self).__name__}.on_play()->{self.prim_path}")
        print(f"PLAY  {self.prim_path}")

        app_window = omni.appwindow.get_default_app_window()
        self.keyboard = app_window.get_keyboard()
        input = carb.input.acquire_input_interface()
        self.keyboard_sub_id = input.subscribe_to_keyboard_events(self.keyboard, self.on_keyboard_input)

        

  




    def on_pause(self):
        carb.log_info(f"{type(self).__name__}.on_pause()->{self.prim_path}")
        input = carb.input.acquire_input_interface()
        input.unsubscribe_to_keyboard_events(self.keyboard, self.keyboard_sub_id)
        self.keyboard_sub_id = None




    def on_stop(self):
        carb.log_info(f"{type(self).__name__}.on_stop()->{self.prim_path}")
        print(f"STOP  {self.prim_path}")

        input = carb.input.acquire_input_interface()
        input.unsubscribe_to_keyboard_events(self.keyboard, self.keyboard_sub_id)
        self.keyboard_sub_id = None



    def on_update(self, current_time: float, delta_time: float):
        carb.log_info(f"{type(self).__name__}.on_update({current_time}, {delta_time})->{self.prim_path}")



    def on_keyboard_input(self, e):

        # print(f"TECLADO  {self.prim_path}")
        # print(e.input)
        # print(e.type)

        abejorro_prim = self.stage.GetPrimAtPath("/World/abejorro")
        torque_atr = abejorro_prim.GetAttribute("physxForce:torque")
        
        if e.input == carb.input.KeyboardInput.O:
            if e.type == carb.input.KeyboardEventType.KEY_PRESS:
                torque = (0,0,0.01)
            if e.type == carb.input.KeyboardEventType.KEY_REPEAT:
                torque = (0,0,0.01)
            if e.type == carb.input.KeyboardEventType.KEY_RELEASE:
                torque = (0,0,0)
            torque_atr.Set(torque)

        if e.input == carb.input.KeyboardInput.P:
            if e.type == carb.input.KeyboardEventType.KEY_PRESS:
                torque = (0,0,-0.01)
            if e.type == carb.input.KeyboardEventType.KEY_REPEAT:
                torque = (0,0,-0.01)
            if e.type == carb.input.KeyboardEventType.KEY_RELEASE:
                torque = (0,0,0)
            torque_atr.Set(torque)            