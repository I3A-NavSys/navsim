
% Lanzar el simulador con el comando:  $ gazebo tatami.world

% Inicialización
% clear; clc;
run('../tools/NAVSIM_PATHS');
% 
% 
% 
% builder = SimpleBuilder('builder',NAVSIM_MODELS_PATH);
% 
% builder.DeployModel('DCmodels/base_drone', ...
%     'vertiport', [-3 -3 0.26], [0 0 0]);
% 
% builder.DeployModel('DCmodels/frame_red', ...
%     'frame_red', [3 2 2], [0 0 1]);
% 
% builder.DeployModel('DCmodels/frame_green', ...
%     'frame_green', [-2 1 0.5], [0 0 2]);
% 
% builder.DeployModel('DCmodels/frame_blue', ...
%     'frame_blue', [3 -1 1], [0 0 3]);
% 
% 
% 
% 
% 
op = DC_Operator("DC_Operator",NAVSIM_MODELS_PATH);
% op.DeployUAV('abejorroMATLAB', [-3 -3 2], [0 0 1]);


op.RemoteCommand('abejorro',true,0.9,0,0,0.1);


op.GetTime()
op.ResetTime;

% pause(1);
% op.RemoveUAV('abejorronnnnn');
% 
% pause(1);
% op.RemoveUAV('abejorro');
