clc
clear
pause(0.1) %ROS2 requires time to clear resources
run('../../../tools/NAVSIM_PATHS');


operator = USpaceOperator('operator',NAVSIM_MODELS_PATH);

operator.DeployUAV(UAVmodels.MiniDroneFP1,'UAV', ...
    [ -190.00  -119.00  +048.10], ...
    [0 0 pi/2 ]);

waitForServer(operator.rosCli_DeployUAV,"Timeout",1);


rosPub1 = ros2publisher(operator.rosNode, ...
    '/NavSim/UAV/RemoteCommand',      ...
    "navsim_msgs/RemoteCommand");


waitForServer(operator.rosCli_DeployUAV,"Timeout",1);


rosPub2 = ros2publisher(operator.rosNode, ...
    "/NavSim/UAV/FlightPlan",      ...
    "navsim_msgs/FlightPlan");

waitForServer(operator.rosCli_DeployUAV,"Timeout",1);



