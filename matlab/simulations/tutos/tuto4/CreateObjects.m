clc
clear

run('../../../tools/NAVSIM_PATHS');

builder  = SimpleBuilder ('builder' ,NAVSIM_MODELS_PATH);
operator = USpaceOperator('operator',NAVSIM_MODELS_PATH);
monitor  = SimpleMonitor('monitor');


% -------------
% Set vertiports

%               x        y        z       rz
portsLoc = [ -190.00  -119.00  +048.00    pi/4
             -152.00  -106.00  +049.00    pi/4
             -134.00  -190.00  +048.00    00
             -092.00  -144.00  +041.00    00
             -074.00  -100.00  +043.00    00
             -073.00  +216.00  +027.00    00
             -007.00  +015.00  +043.00    00
             +060.00  +131.00  +032.00    00
             +180.00  +033.00  +050.00    00
             +186.00  -081.00  +050.00    pi/2
             -200.00  +157.00  +044.00    pi/2
             -200.00   +20.00  +042.00    pi/2
             +186.00  +195.00  +039.00    pi/2
             +126.00  -189.00  +039.00    pi/2
           ];


for i = 1:size(portsLoc,1)
   
    UAVid = sprintf('BASE%02d', i);
    builder.DeployModel('UAM/vertiport_H', UAVid, ...
        portsLoc(i,1:3), ...
        [0 0 portsLoc(i,4)]);
    operator.SetVertiport(UAVid,portsLoc(i,1:3),1);
end


% -------------
% Deploy fleet
info = UAVinfo('',UAVmodels.MiniDroneFP1);
info.velMax = 10;
operator.DeployFleet(size(portsLoc,1),info);
% operator.DeployUAV(info,'UAV01', ...
%     operator.VPs(1).pos+[0 0 0.20], ...
%     [0 0 0]);



pause(1)

%%
% -------------
% Begin operations
for UAVid = operator.FleetIds
    monitor.TrackUAV(UAVid); 
    operator.OperateUAV(UAVid);
end



%% 
% pause(1)
op = operator.ops(27);
monitor.PositionFigure(op.UAVid,op.fp);
monitor.VelocityFigure(op.UAVid,op.fp);