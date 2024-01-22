% Solicitamos el tiempo simulado al plugin the mundo ejecutado en Gazebo
clc
clear

rosNode = ros2node('Tester');
rosSub_Time = ros2subscriber(rosNode, ...
    '/World/Time','builtin_interfaces/Time');

[msg,status,~] = receive(rosSub_Time,1);
if (status)
    sec = double(msg.sec);
    mil = double(msg.nanosec / 1E6);

    disp(['Simulation time is ' num2str(sec) '.' num2str(mil)])
else
    sec = 0;
    mil = 0;
end

