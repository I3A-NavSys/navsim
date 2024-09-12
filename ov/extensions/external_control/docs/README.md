# NavSim external input control

This is an extension built to give support to external input controllers.
This extensions only recognizes prims which have an extra property (you have to add it to the prims you want) named 'NavSim:Manipulable'.
It is event driven, that is, we use events and the bus message event stream to communicate with control behavior of the prim.

IMPORTANT:
This extension does not implement any kind of control logic for the prims. It only recognizes a joystick controller and the keyboard, and sends its inputs to the script which is subscribed to our custom 'UAV_EVENT' (got from carb.events.type_from_string("NavSim." + str(prim.GetPath())))

JOYSTICKS SUPPORTED (others may work but it is no guaranteed)
- Thrustmaster Flight Stick X

KEYBOARD MAPPING:
'W' = Move fordwards
'A' = Move leftside
'S' = Move backwards
'D' = Move rightside

'Top arrow key' = Go up
'Left arrow key' = Turn left
'Down arrow key' = Go down
'Right arrow key' = Go right

'Space' = Turn on/off rottors
'NUMPAD_SUBTRACT' = Change active camera (left)
'NUMPAD_ADD' = Change active camera (right)
'NUMPAD_8' = Increase follow height camera (DroneCamera)
'NUMPAD_4' = Decrease follow distance camera (DroneCamera)
'NUMPAD_6' = Increase follow distance camera (DroneCamera)
'NUMPAD_2' = Decrease follow height camera (DroneCamera)
