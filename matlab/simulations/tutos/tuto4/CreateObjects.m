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
    operator.SetVertiport(UAVid,portsLoc(i,1:3));
end


% -------------
% Deploy fleet
operator.DeployFleet(1,UAVmodels.MiniDroneFP1);

%%
for UAVid = operator.Fleet
    monitor.TrackUAV(UAVid); 
end


operator.OperateUAV("UAV01");
