clc
clear


% ROS2 node
rosNode = ros2node('operatorTEST');





rosCli_DeployUAV = ros2svcclient(rosNode, ...
    '/NavSim/DeployModel','navsim_msgs/DeployModel', ...
    'History','keepall');



% waitForServer(rosCli_DeployUAV,"Timeout",1);


rosPub2 = ros2publisher(rosNode, ...
    "/NavSim/UAV/FlightPlan",      ...
    "navsim_msgs/FlightPlan",...
    'History','keepall');


rosPub = ros2publisher(rosNode, ...
    '/NavSim/UAV/RemoteCommand',      ...
    "navsim_msgs/RemoteCommand");

rosPub2 = ros2publisher(rosNode, ...
    '/NavSim/UAV2/RemoteCommand',      ...
    "navsim_msgs/RemoteCommand");


waitForServer(rosCli_DeployUAV,"Timeout",1);



