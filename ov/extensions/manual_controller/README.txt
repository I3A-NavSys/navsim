Steps to control the UAV via joystick:
    - Open the scene '/fleet/iker_joystick_quadcopter/quadcopter.usd' to try Iker controller
    - Open the scene '/fleet/rafa_joystick_quadcopter/UAM_minidrone.usd' to try Rafa controller
    - Click on 'Yes' when asked about attached behaviors scripts
    - If external_control extension is not enabled, enable it
    - Click on 'REFRESH' button from the extension to get the manipulable UAV
    - Then start the simulation so that the behavior script start running
    - Finally click on 'START' button from the extension to begin controlling the UAV via joystick (or even keyboard)

NOTES:
    - The joystick controller must be plugged in the pc, you can do it during simulation, it should work
    - Each time that you close or open a new scene, or create a new manipulable UAV, click on 'REFRESH' button so that the extension can reference the correct prim
