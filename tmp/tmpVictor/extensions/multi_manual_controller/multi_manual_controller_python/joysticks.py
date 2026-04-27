import numpy as np
import logging
import pygame

class Joysticks:
    def __init__(self):
        self.joysticks_inputs = {}
        self.joysticks: list[pygame.joystick.JoystickType] = []
        self.joysticks_ids = []
        self.logger = logging.getLogger("A_joy")
        self.initialized = False

    def start(self):
        if not self.initialized:
            self.initialized = True

            # Initialize pygame environment
            pygame.init()
            pygame.joystick.init()

    def stop(self):
        if self.initialized:
            self.initialized = False
            
            # Finish pygame environment
            pygame.joystick.quit()
            pygame.quit()

        self.joysticks_inputs = {}
        self.joysticks: list[pygame.joystick.JoystickType] = []
        self.joysticks_ids = []

    def get_inputs(self):
        # Check if joystick is connected
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                self.joysticks.append(pygame.joystick.Joystick(event.device_index))
                id = self.joysticks[-1].get_id()
                self.joysticks_inputs[id] = []
                self.joysticks_ids.append(id)

        # Get the joystick inputs
        for joystick in self.joysticks:
            inputs = [0,0,0,0,0,0]

            inputs[0] = round(joystick.get_axis(1), 2)     # Left - Right
            inputs[1] = round(joystick.get_axis(0), 2)     # Fordward - Backward
            inputs[2] = round(joystick.get_axis(3), 2)     # Slider
            inputs[3] = round(joystick.get_axis(2), 2)     # Rotation Left - Right
            inputs[4] = joystick.get_button(0)             # cmd on/off
            inputs[5] = joystick.get_button(1)             # reset position

            # Order when using get_id: top to bottom; right to left; frontal connectors are the last ones
            self.joysticks_inputs[joystick.get_id()] = np.array(inputs)

        return self.joysticks_ids, self.joysticks_inputs

        