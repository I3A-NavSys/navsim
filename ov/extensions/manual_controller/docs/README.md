# NavSim external input control

This is an extension built to give support to external input controllers.
This extensions only recognizes prims which have an extra property (you have to add it to the prims you want) named 'NavSim:Manipulable'.
It is event driven, that is, we use events and the bus message event stream to communicate with control behavior of the prim.

IMPORTANT:
This extension does not implement any kind of control logic for the prims. It only recognizes a joystick controller and the keyboard, and sends its inputs to the script which is subscribed to our custom 'UAV_EVENT' (got from carb.events.type_from_string("NavSim." + str(prim.GetPath())))

JOYSTICKS SUPPORTED (others may work but it is no guaranteed)
- Thrustmaster Flight Stick X

JOYSTICK MAPPING:
'Push joystisk fordwards'   =   Move fordwards
'Push joystick to left'     =   Move leftside
'Push joystisk backwards'   =   Move backwards
'Push joystisk to right'    =   Move rightside

'Slide slider up'           =   Go up
'Slide slider down'         =   Go down
'Rotate joystick to left'   =   Turn left
'Rotate joystick to right'  =   Turn right

'Button 1'                  =   Turn on/off rottors
'Button 5'                  =   Camera zoom in
'Button 6'                  =   Camera zoom out
'Thumb mini joystick up'    =   (Look down) Pitch positive rotation along 'Y' axe from the UAV (VelocityCamera)
'Thumb mini joystick down'  =   (Look up) Pitch negative rotation along 'Y' axe from the UAV (VelocityCamera)
'Thumb mini joystick right' =   (Look left) Yaw positive rotation along 'Z' axe from the UAV (VelocityCamera)
'Thumb mini joystick left'  =   (Look right) Yaw negative rotation along 'Z' axe from the UAV (VelocityCamera)

KEYBOARD MAPPING:
'W'     =   Move fordwards
'A'     =   Move leftside
'S'     =   Move backwards
'D'     =   Move rightside

'Top arrow key'     =   Go up
'Down arrow key'    =   Go down
'Left arrow key'    =   Turn left
'Right arrow key'   =   Go right

'Space'             =   Turn on/off rottors
'NUMPAD_SUBTRACT'   =   Camera zoom in
'NUMPAD_ADD'        =   Camera zoom out
'NUMPAD_8'          =   (Look down) Pitch positive rotation along 'Y' axe from the UAV (VelocityCamera)
'NUMPAD_2'          =   (Look up) Pitch negative rotation along 'Y' axe from the UAV (VelocityCamera)
'NUMPAD_6'          =   (Look left) Yaw positive rotation along 'Z' axe from the UAV (VelocityCamera)
'NUMPAD_4'          =   (Look right) Yaw negative rotation along 'Z' axe from the UAV (VelocityCamera)

'LEFT_CONTROL'   =   Camera zoom in
'RIGHT_CONTROL'        =   Camera zoom out
'I'          =   (Look down) Pitch positive rotation along 'Y' axe from the UAV (VelocityCamera)
'K'          =   (Look up) Pitch negative rotation along 'Y' axe from the UAV (VelocityCamera)
'L'          =   (Look left) Yaw positive rotation along 'Z' axe from the UAV (VelocityCamera)
'J'          =   (Look right) Yaw negative rotation along 'Z' axe from the UAV (VelocityCamera)