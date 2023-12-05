classdef testClient
properties

rosNode
rosCli_Test  % Cliente de servicio para pausar la simulación

end % properties

methods

% Constructor
function obj = testClient()
    % Inicializar la conexión ROS
    obj.rosNode = ros2node('TestClient');


    % Crear clientes de servicio
    obj.rosCli_Test = ros2svcclient(obj.rosNode, ...
        '/AirSpace/Test','utrafman_msgs/Test');
end


% llamar al servicio Test
function callTest(obj)
    
    req = ros2message(obj.rosCli_Test);
    req.a = int16(2);
    req.b = int16(3);
    
    status = waitForServer(obj.rosCli_Test,"Timeout",3);
    res = call(obj.rosCli_Test,req,"Timeout",3);
    
    if res.sum == 5
        disp("El servicio se ha llamado correctamente.")
    end

end


end % methods
end % classdef
