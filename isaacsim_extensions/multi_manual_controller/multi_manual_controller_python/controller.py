import numpy as np
from .joysticks import Joysticks


class Controller:
    """Processes joystick inputs and converts them into linear and angular UAV control commands."""

    def __init__(self):
        self.is_running = False
        self.linear_vel_limit = 6.0
        self.ang_vel_limit = 0.7
        self.joysticks = Joysticks()
        self.uav_control = None

        self.uavs_rottors_on_change = None
        self.uavs_current_rottors_on = None
        self.uavs_reset_button_prev = None

    def start(self, uav_control):
        """Initializes controller state arrays matched to the UAV prim count."""
        if not self.is_running:
            self.uav_control = uav_control
            uav_count = self.uav_control.rigid_prim_view.count

            # Edge detection and state tracking arrays for buttons
            self.uavs_rottors_on_change = np.zeros(uav_count, dtype=bool)
            # Default rotors to ON so UAVs respond immediately to user input
            self.uavs_current_rottors_on = np.ones(uav_count, dtype=bool)
            # Edge detection for reset button to avoid continuous reset triggers while held
            self.uavs_reset_button_prev = np.zeros(uav_count, dtype=bool)

            self.joysticks.start()
            self.is_running = True

    def stop(self):
        """Stops joystick polling and marks controller as inactive."""
        if self.is_running:
            self.joysticks.stop()
            self.is_running = False

    def control(self, uav_physics_indices: list[int] | None = None) -> tuple[np.ndarray, np.ndarray, int]:
        """
        Polls joystick inputs and generates commanded velocities for active UAVs.

        Args:
            uav_physics_indices: List mapping joystick order to actual physics buffer indices.

        Returns:
            Tuple of (cmd_local_linear_vel of shape (M, 3), cmd_yaw_rotation of shape (M,), amount_joysticks).
        """
        if not self.is_running or self.uav_control is None:
            return np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.float32), 0

        joysticks_ids, joysticks_inputs = self.joysticks.get_inputs()
        total_joysticks = len(joysticks_ids)
        total_uavs = self.uav_control.rigid_prim_view.count

        # Bound by available UAVs to prevent index errors
        amount_joysticks = min(total_joysticks, total_uavs)
        if uav_physics_indices is not None:
            amount_joysticks = min(amount_joysticks, len(uav_physics_indices))

        if amount_joysticks == 0:
            return np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.float32), 0

        cmd_local_linear_vel = np.zeros((amount_joysticks, 3), dtype=np.float32)
        cmd_yaw_rotation = np.zeros(amount_joysticks, dtype=np.float32)

        for i in range(amount_joysticks):
            joy_id = joysticks_ids[i]
            joy_input = joysticks_inputs.get(joy_id)
            if joy_input is None:
                continue

            rottors_button = joy_input[4]
            reset_button = joy_input[5]

            # Physics index mapping for the target UAV
            target_phys_idx = uav_physics_indices[i] if uav_physics_indices is not None else i

            # Edge-triggered position reset (triggers once upon press, not continuously while held)
            if reset_button == 1.0 and not self.uavs_reset_button_prev[i]:
                self.uavs_reset_button_prev[i] = True
                self.uav_control.reset_pose([target_phys_idx])
            elif reset_button == 0.0:
                self.uavs_reset_button_prev[i] = False

            # Edge-triggered rotor state toggling (Button 0)
            if rottors_button == 1.0 and not self.uavs_rottors_on_change[i]:
                self.uavs_rottors_on_change[i] = True
                self.uavs_current_rottors_on[i] = not self.uavs_current_rottors_on[i]
            elif rottors_button == 0.0:
                self.uavs_rottors_on_change[i] = False

            # Assign command only if rotors are active for this UAV
            if self.uavs_current_rottors_on[i]:
                cmd_local_linear_vel[i] = joy_input[:3] * -self.linear_vel_limit
                cmd_yaw_rotation[i] = joy_input[3] * -self.ang_vel_limit

        return cmd_local_linear_vel, cmd_yaw_rotation, amount_joysticks

    def check_joysticks(self) -> dict[int, np.ndarray]:
        """Provides raw joystick inputs for UI visualization."""
        _, joysticks_inputs = self.joysticks.get_inputs()
        return joysticks_inputs
