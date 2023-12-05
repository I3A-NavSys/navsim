clc
clear

rosNode = ros2node('TestClient');
[rosCli_Test,req] = ros2svcclient(rosNode, ...
        '/AirSpace/Test','utrafman_msgs/Test');
    
% req = ros2message(obj.rosCli_Test);
req.a = int16(2);
req.b = int16(3);
    
status = waitForServer(rosCli_Test,"Timeout",3);
if ~status
    error("Es servicio ROS no está disponible")
end
res = call(rosCli_Test,req,"Timeout",3);
    
if res.sum == 5
    disp("El servicio se ha llamado correctamente.")
end