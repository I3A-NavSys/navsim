
% Lanzar el simulador con el comando:  $ gazebo tatami.world

% Inicialización
clear; clc;
run('../tools/NAVSIM_PATHS');



builder = SimpleBuilder('builder',NAVSIM_MODELS_PATH);

builder.DeployModel('DCmodels/base_drone', ...
    'vertiport', [-3 -3 0.26], [0 0 0]);

builder.DeployModel('DCmodels/frame_red', ...
    'frame_red', [3 2 2], [0 0 1]);

builder.DeployModel('DCmodels/frame_green', ...
    'frame_green', [-2 1 0.5], [0 0 2]);

builder.DeployModel('DCmodels/frame_blue', ...
    'frame_blue', [3 -1 1], [0 0 3]);





op = DC_Operator("DC_Operator",NAVSIM_MODELS_PATH);
% op.DeployUAV('abejorroMATLAB', [-3 -3 2], [0 0 1]);
% op.RemoteCommand('abejorro1',true,0,0,0.1,0.2);
 
while (true)

    msg = receive(op.rosSub_GetImage,3);
    img2D = uint8(zeros(msg.height,msg.width,3));
    elem = 1;
    for fil = 1:msg.height
        for col = 1:msg.width
            for rgb = 1:3
                img2D(fil,col,rgb) = msg.data(elem);
                elem = elem + 1;
            end
        end
    end
    imshow(img2D)
    pause(1)
end




% op.DeployUAV('abejorro1',[-4.5 -4.5 1],[0 0 0]);
op.RemoteCommand('UAV00',true,0,0,0.1,0);
pause(1);
op.RemoteCommand('UAV00',true,1,0,0,1);
pause(8);
op.RemoteCommand('UAV00',false,0,0,0,0);



% op.GetTime()
% op.ResetTime;

% pause(1);
% op.RemoveUAV('abejorronnnnn');
% 
% pause(1);
% op.RemoveUAV('abejorro');
