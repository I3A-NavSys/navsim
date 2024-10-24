try:
    import pygame
except:
    raise Exception("ERROR: 'pygame' package is not installed. Copy and paste in the Script Editor the " +
                    "folllowing code\n\n" + 
                    "# -- START CODE ------------------------------\n" +
                    "import omni.kit.pipapi\n" +
                    "omni.kit.pipapi.install(\"pygame\")\n" +
                    "# -- END CODE --------------------------------\n")

class Joysticks:
    def __init__(self):
        self.joysticks_inputs = {}
        self.joysticks: list[pygame.joystick.JoystickType] = []
        self.joysticks_ids = []

    def start(self):
        # Initialize pygame environment
        pygame.init()
        pygame.joystick.init()

    def stop(self):
        # Finish pygame environment
        pygame.joystick.quit()
        pygame.quit()

    def get_inputs(self):
        # Check if joystick is connected
        for event in pygame.event.get():
            if event.type == pygame.JOYDEVICEADDED:
                self.joysticks.append(pygame.joystick.Joystick(event.device_index))
                self.joysticks_inputs[event.device_index] = []
                self.joysticks_ids.append(event.device_index)

        # Get the joystick inputs
        for joystick in self.joysticks:
            inputs = [0,0,0,0,0]

            inputs[0] = round(joystick.get_axis(1), 2)     # Left - Right
            inputs[1] = round(joystick.get_axis(0), 2)     # Fordward - Backward
            inputs[2] = round(joystick.get_axis(3), 2)     # Slider
            inputs[3] = round(joystick.get_axis(2), 2)     # Rotation Left - Right
            inputs[4] = joystick.get_button(0)             # cmd on/off

            self.joysticks_inputs[joystick.get_instance_id()] = inputs

        return self.joysticks_ids, self.joysticks_inputs

        