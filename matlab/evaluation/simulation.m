
clear builder operator monitor;
clc

% Simulation

builder  = SimpleBuilder ('builder' ,NAVSIM_MODELS_PATH);
pause(0.1)
operator = USpaceOperator('operator',NAVSIM_MODELS_PATH);
pause(0.1)
monitor  = SimpleMonitor('monitor');
pause(0.1)

% vertiports
for i = 1:size(portsLoc,1)
   
    id = sprintf('BASE%02d', i);
    builder.DeployModel('UAM/vertiport_H', id, ...
        portsLoc(i,1:3), ...
        [0 0 portsLoc(i,4)]);
    operator.SetVertiport(id,portsLoc(i,1:3),1);

end

% -------------
% UAV performance
info = UAVinfo('',UAVmodels.MiniDroneFP1);
info.maxForwardVel   = 12;    % m/s
info.maxForwardAcel  =  1;    % m/s2
info.maxVerticalVel  =  3;    % m/s  
info.maxVerticalAcel =  1;    % m/s2
info.maxAngularVel   =  5;    % rad/s
info.maxAngularAcel  =  1;    % rad/s2


% Deploy UAVs

% operator.DeployUAV(info,'UAV01', ...
%     [ -190.00  -119.00  +048.10 ],...
%     [    0.00     0.00     pi/4  ]);
% monitor.TrackUAV('UAV01');
% 
% operator.DeployUAV(info,'UAV02', ...
%     [ -189.50  -119.00  +048.10 ],...
%     [    0.00     0.00     pi/4 ]);
% monitor.TrackUAV('UAV02');


% operator.DeployUAV(info,'UAV03', ...
%     [ -200.00  -176.00  +010.10 ],...
%     [    0.00     0.00     pi/4 ]);
% monitor.TrackUAV('UAV03');
% 
% operator.DeployUAV(info,'UAV04', ...
%     [ -199.50  -176.00  +010.10 ],...
%     [    0.00     0.00     pi/4 ]);
% monitor.TrackUAV('UAV04');


% operator.DeployUAV(info,'UAV05', ...
%     [ -200.00  -176.00  +010.10 ],...
%     [    0.00     0.00     pi/4 ]);
% monitor.TrackUAV('UAV05');
% 
% operator.DeployUAV(info,'UAV06', ...
%     [ -199.50  -176.00  +010.10 ],...
%     [    0.00     0.00     pi/4 ]);
% monitor.TrackUAV('UAV06');


operator.DeployUAV(info,'UAV07', ...
    [ -200.00  -176.00  +010.10 ],...
    [    0.00     0.00     pi/4 ]);
monitor.TrackUAV('UAV07');

operator.DeployUAV(info,'UAV08', ...
    [ -199.50  -176.00  +010.10 ],...
    [    0.00     0.00     pi/4 ]);
monitor.TrackUAV('UAV08');


% operator.DeployUAV(info,'UAV09', ...
%     [ -200.00  -085.00  +029.10 ],...
%     [    0.00     0.00     pi/4 ]);
% monitor.TrackUAV('UAV09');
% 
% operator.DeployUAV(info,'UAV10', ...
%     [ -199.50  -085.00  +029.10 ],...
%     [    0.00     0.00     pi/4 ]);
% monitor.TrackUAV('UAV10');


%% fp1 & fp2

% time = operator.GetTime();
% 
% fp1.RescheduleAt(time + 10);
% operator.SendFlightPlan('UAV01',fp1);
% 
% pause(0.2);
% 
% fp2.RescheduleAt(time + 10.2);
% operator.SendFlightPlan('UAV02',fp2);
% 
% operator.WaitTime(max(fp1.FinishTime,fp2.FinishTime));
% monitor.PositionFigure('UAV01',fp1,0.01);
% monitor.VelocityFigure('UAV01',fp1,0.01);
% monitor.PositionFigure('UAV02',fp2,0.01);
% monitor.VelocityFigure('UAV02',fp2,0.01);

%% fp3 & fp4

% time = operator.GetTime();
% 
% fp3.RescheduleAt(time + 10);
% operator.SendFlightPlan('UAV03',fp3);
% 
% pause(0.2);
% 
% fp4.RescheduleAt(time + 10.2);
% operator.SendFlightPlan('UAV04',fp4);
% 
% operator.WaitTime(max(fp3.FinishTime,fp4.FinishTime));
% monitor.PositionFigure('UAV03',fp3,0.01);
% monitor.VelocityFigure('UAV03',fp3,0.01);
% monitor.PositionFigure('UAV04',fp4,0.01);
% monitor.VelocityFigure('UAV04',fp4,0.01);


%% fp5 & fp6

% time = operator.GetTime();
% 
% fp5.RescheduleAt(time + 10);
% operator.SendFlightPlan('UAV05',fp5);
% 
% pause(0.2);
% 
% fp6.RescheduleAt(time + 10.2);
% operator.SendFlightPlan('UAV06',fp6);
% 
% operator.WaitTime(max(fp5.FinishTime,fp6.FinishTime));
% monitor.PositionFigure('UAV05',fp5,0.01);
% monitor.VelocityFigure('UAV05',fp5,0.01);
% monitor.PositionFigure('UAV06',fp6,0.01);
% monitor.VelocityFigure('UAV06',fp6,0.01);

%% fp7 & fp8

time = operator.GetTime();

fp7.RescheduleAt(time + 10);
operator.SendFlightPlan('UAV07',fp7);

pause(0.2);

fp8.RescheduleAt(time + 10.2);
operator.SendFlightPlan('UAV08',fp8);

operator.WaitTime(max(fp7.FinishTime,fp8.FinishTime));
monitor.PositionFigure('UAV07',fp7,0.01);
monitor.VelocityFigure('UAV07',fp7,0.01);
monitor.PositionFigure('UAV08',fp8,0.01);
monitor.VelocityFigure('UAV08',fp8,0.01);

%% fp9 & fp10

% time = operator.GetTime();
% 
% fp9.RescheduleAt(time + 10);
% operator.SendFlightPlan('UAV09',fp9);
% 
% pause(1);
% 
% fp10.RescheduleAt(time + 11);
% operator.SendFlightPlan('UAV10',fp10);
% 
% operator.WaitTime(max(fp9.FinishTime,fp10.FinishTime));
% monitor.PositionFigure('UAV09',fp9,0.01);
% monitor.VelocityFigure('UAV09',fp9,0.01);
% monitor.PositionFigure('UAV10',fp10,0.01);
% monitor.VelocityFigure('UAV10',fp10,0.01);
% 
% operator.PauseSim;
