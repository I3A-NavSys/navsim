clear; 
clc;

run('../../../tools/NAVSIM_PATHS');

%-------------
%Create Flight Plan

%             t      x    y    z     vx vy vz
way_data = [ 15   -190 -119 +048.6   00 00 00       % 0.7 m/s
             20   -190 -119 +052.0   00 00 00       % 4.0 m/s
             30   -152 -106 +052.0   00 00 00       % 0.5 m/s
             35   -152 -106 +049.6   00 00 00 ];


fp1  = FlightPlan(1,Waypoint.empty);
for i = 1:size(way_data,1)
    wp = Waypoint();
    wp.t = way_data(i,1);
    wp.setPosition(way_data(i,2:4));
    fp1.setWaypoint(wp);
end

% Display
time_step = 0.1;
fp1.routeFigure(time_step,'b')
fp1.velocityFigure(time_step,'b')

%% -------------
% Deploy fleet

builder  = SimpleBuilder ('builder' ,NAVSIM_MODELS_PATH);
builder.DeployModel('DC/base_drone','BASE1',[-190 -119 48.25],[0 0 0]);
builder.DeployModel('DC/base_drone','BASE2',[-152 -106 49.25],[0 0 0]);
% builder.DeployModel('DC/base_drone','BASE3',[180 33 50.25],[0 0 0]);

operator = USpaceOperator('operator',NAVSIM_MODELS_PATH);
operator.DeployUAV(UAVmodels.MiniDroneFP1,'UAV01',[-190 -119 48.6],[0 0 0]);
pause(2)

%% -------------
% run monitoring service

monitor = SimpleMonitor('monitor');
monitor.trackUAV('UAV01');



%%

operator.SendFlightPlan('UAV01',fp1);



%%
monitor.positionFigure('UAV01',fp1,1);



% operator.RemoveUAV('UAV01');

