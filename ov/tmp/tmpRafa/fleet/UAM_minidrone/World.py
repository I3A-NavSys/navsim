from omni.kit.scripting import BehaviorScript
import omni.kit
from omni.isaac.core.utils import prims
from omni.isaac.core import SimulationContext


import carb
import omni.appwindow
import carb.input
import omni.kit.scripting



class World(BehaviorScript):


    
    def on_init(self):
        print("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
        print(f"INIT    {self.prim_path}")





    def on_play(self):

        print(f"PLAY    {self.prim_path}")

        self.abejorro_prim = self.stage.GetPrimAtPath("/World/abejorro")
        self.torque_atr = self.abejorro_prim.GetAttribute("physxForce:torque")

        app_window = omni.appwindow.get_default_app_window()
        self.keyboard = app_window.get_keyboard()
        self.keyboard_sub_id = None

        input = carb.input.acquire_input_interface()
        self.keyboard_sub_id = input.subscribe_to_keyboard_events(self.keyboard, self.on_keyboard_input)

          

    def on_pause(self):
        input = carb.input.acquire_input_interface()
        input.unsubscribe_to_keyboard_events(self.keyboard, self.keyboard_sub_id)
        self.keyboard_sub_id = None
    



    def on_stop(self):
        print(f"STOP    {self.prim_path}")

        input = carb.input.acquire_input_interface()
        input.unsubscribe_to_keyboard_events(self.keyboard, self.keyboard_sub_id)
        self.keyboard_sub_id = None



    def on_keyboard_input(self, e):

        # print(f"TECLADO  {self.prim_path}")
        # print(e.input)
        # print(e.type)


        
        if e.input == carb.input.KeyboardInput.O:
            if e.type == carb.input.KeyboardEventType.KEY_PRESS:
                print(e.input)


                abejorro_prim = self.stage.GetPrimAtPath("/World/abejorro")
                torque_atr = abejorro_prim.GetAttribute("physxForce:torque")


                torque = (0,0,0.1)
                torque_atr.Set(torque)
            if e.type == carb.input.KeyboardEventType.KEY_RELEASE:
                # torque = (0,0,0)
                # self.torque_atr.Set(torque)

        if e.input == carb.input.KeyboardInput.P:
            if e.type == carb.input.KeyboardEventType.KEY_PRESS:
                print(e.input)
                torque = (0,0,-0.1)
                self.torque_atr.Set(torque)            
            if e.type == carb.input.KeyboardEventType.KEY_RELEASE:
                torque = (0,0,0)
                self.torque_atr.Set(torque)            

        if e.input == carb.input.KeyboardInput.A:         
            if e.type == carb.input.KeyboardEventType.KEY_RELEASE:
                print ("accion")
 
 