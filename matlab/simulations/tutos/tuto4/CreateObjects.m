clc
clear
run('../../../tools/NAVSIM_PATHS');

builder  = SimpleBuilder ('builder' ,NAVSIM_MODELS_PATH);
operator = USpaceOperator('operator',NAVSIM_MODELS_PATH);
monitor  = SimpleMonitor('monitor');


% -------------
% Set vertiports

%               x        y        z       rz
portsLoc = [ -190.00  -119.00  +048.00    pi/2
             -152.00  -106.00  +049.00    pi/2
             +180.00  +033.00  +050.00    00
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
operator.DeployFleet(3,info);

%%
for UAVid = operator.FleetIds
    monitor.TrackUAV(UAVid); 
end

% crear una función que opere a todos!!!
operator.OperateUAV("UAV01");
operator.OperateUAV("UAV02");
operator.OperateUAV("UAV03");


% monitor.PositionFigure('UAV01',operator.ops(1).fp);
% monitor.VelocityFigure('UAV01',operator.ops(1).fp);