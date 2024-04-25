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
   
    id = sprintf('BASE%02d', i);
    builder.DeployModel('UAM/vertiport_H', id, ...
        portsLoc(i,1:3), ...
        [0 0 portsLoc(i,4)]);
end


%% -------------
% Deploy fleet

%               x        y        z       rz
fleetLoc = [ -190.00  -119.00  +048.10    pi/2
             -152.00  -106.00  +049.10    pi/2
             +180.00  +033.00  +050.10    -0.9*pi
           ];


info = UAVinfo('',UAVmodels.MiniDroneFP1);
info.maxForwardVel = 10;

for i = 1:size(fleetLoc,1)
   
    id = sprintf('UAV%02d', i);
    operator.DeployUAV(info,id, ...
        fleetLoc(i,1:3), ...
        [0 0 fleetLoc(i,4)]);
    monitor.TrackUAV(id);
    
end

