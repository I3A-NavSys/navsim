% Solicitamos el tiempo simulado al plugin the mundo ejecutado en Gazebo
clc
clear

rosNode = ros2node('Tester');
rosCli_Time = ros2svcclient(rosNode,'/World/Time','navsim_msgs/Time');

req = ros2message(rosCli_Time);
req.reset = false;
status = waitForServer(rosCli_Time,"Timeout",3);
if ~status
    error("Es servicio ROS2 no está disponible")
end
res = call(rosCli_Time,req,"Timeout",3);

disp("El tiempo de simulación es")
disp(res.time.sec)
disp(res.time.nanosec)
