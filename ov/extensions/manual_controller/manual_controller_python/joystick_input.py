try:
    import pygame
except:
    raise Exception("ERROR: 'pygame' package is not installed. Copy and paste in the Script Editor the folllowing code\n\n" + 
                    "# -- START CODE ------------------------------\n" +
                    "import omni.kit.pipapi\n" +
                    "omni.kit.pipapi.install(\"pygame\")\n" +
                    "# -- END CODE --------------------------------\n")

import asyncio
import numpy as np

from omni.isaac.core.utils.stage import get_current_stage

from pxr import UsdGeom, Gf, Usd

class JoystickInput:
    def __init__(self):
        self.inputs = [0,0,0,0,0,0,0,0,0]
        self.event = asyncio.Event()

    def start(self):
        # Loop condition
        self._stop = False

        # Initialize pygame environment
        pygame.init()
        pygame.joystick.init()
        
        # Start input coroutine
        asyncio.ensure_future(self.input())


    def stop(self):
        self._stop = True
        self.event.set()

        # Finish pygame environment
        pygame.joystick.quit()
        pygame.quit()


    def ask_input(self):
        self.event.set()


    async def input(self):
        joystick = None

        while not self._stop:
            await self.event.wait()

            # Avoid exception when saving file while running simulation
            if self._stop:
                break

            # Check if joystick is connected
            for event in pygame.event.get():
                if event.type == pygame.JOYDEVICEADDED:
                    joystick = pygame.joystick.Joystick(event.device_index)

            if joystick != None:
                # Joystick axis
                self.inputs[0] = round(joystick.get_axis(1), 2)     # Left - Right
                self.inputs[1] = round(joystick.get_axis(0), 2)     # Fordward - Backward
                self.inputs[2] = round(joystick.get_axis(3), 2)     # Slider
                self.inputs[3] = round(joystick.get_axis(2), 2)     # Rotation Left - Right

                # Joystick buttons
                self.inputs[4] = joystick.get_button(0)             # cmd on/off
                self.inputs[5] = joystick.get_button(4)*-1          # -1 active camera
                self.inputs[6] = joystick.get_button(5)             # +1 active camera
                self.inputs[7] = joystick.get_hat(0)[0]             # Follow distance
                self.inputs[8] = joystick.get_hat(0)[1]             # Follow height

            self.event.clear()