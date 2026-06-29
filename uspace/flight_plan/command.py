class Command:
    def __init__(
        self, 
        on: bool = False, 
        vel_x: float = 0, 
        vel_y: float = 0, 
        vel_z: float = 0, 
        rot_z: float = 0, 
        duration: float = 0
    ):
        self.set(on, vel_x, vel_y, vel_z, rot_z, duration)

    def __repr__(self):
        return (
            "Command Information:"
            f"\nON: {self.on}"
            f"\nvel_x: {self.vel_x:.3f}"
            f"\nvel_y: {self.vel_y:.3f}"
            f"\nvel_z: {self.vel_z:.3f}"
            f"\nrot_z: {self.rot_z:.3f}"
            f"\nduration: {self.duration:.3f}"
        )

    def set(
        self, on: bool, 
        vel_x: float, 
        vel_y: float, 
        vel_z: float, 
        rot_z: float, 
        duration: float
    ) -> None:
        """
        Sets the command parameters.

        Args:
            on (bool) : Whether the rotors should be active or not.
            vel_x (float) : Linear velocity desired in X axis (m/s).
            vel_y (float) : Linear velocity desired in Y axis (m/s).
            vel_z (float) : Linear velocity desired in Z axis (m/s).
            rot_z (float) : Angular velocity desired in Z axis (rad/s).
            duration (float) : Duration of the command (s).
        """

        self.on   = on
        self.vel_x = vel_x
        self.vel_y = vel_y
        self.vel_z = vel_z
        self.rot_z = rot_z
        self.duration = duration

    def off(self, duration: float = 0):
        """
        Turns off the command.
        """

        self.set(False, 0, 0, 0, 0, duration)

    def hover(self, duration: float = 10):
        """
        Sets the command to hover (on=True, velocities=0).
        """

        self.set(True, 0, 0, 0, 0, duration)
