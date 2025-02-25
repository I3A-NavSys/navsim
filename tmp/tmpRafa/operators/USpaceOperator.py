class USpaceOperator:

    name = ""           # Operator name
    is_running : bool   # Simulation state


    def __init__(self, name):
        self.name = name
        self.is_running = False



    def play_simulation(self):
        if not self.is_running:
            self.is_running = True
            print(f"Simulation '{self.name}' started.")
        else:
            print(f"Simulation '{self.name}' is already running.")



    def pause_simulation(self):
        if self.is_running:
            self.is_running = False
            print(f"Simulation '{self.name}' paused.")
        else:
            print(f"Simulation '{self.name}' is not running.")