clear; 
clc;

run('../../../tools/NAVSIM_PATHS');


builder  = SimpleBuilder ('builder' ,NAVSIM_MODELS_PATH);
builder.DeployModel('DC/base_drone','BASE1',[-190 -119 48.25],[0 0 0]);
builder.DeployModel('DC/base_drone','BASE2',[-152 -106 49.25],[0 0 0]);
% builder.DeployModel('DC/base_drone','BASE3',[180 33 50.25],[0 0 0]);



operator = USpaceOperator('operator',NAVSIM_MODELS_PATH);
operator.DeployUAV(UAVmodels.MiniDroneFP1,'UAV01',[-190 -119 48.6],[0 0 0]);
pause(2)


%%


monitor = SimpleMonitor('monitor');
monitor.trackUAV('UAV01');



%%
% [s,~] = operator.GetTime();

fp2 = [ 
         -190 -119 48.6   0 0 0   15    % 0.7 m/s
         -190 -119 52.0   0 0 0   20    % 4.0 m/s
         -152 -106 52.0   0 0 0   30    % 0.5 m/s
         -152 -106 49.6   0 0 0   35    
      ];

operator.SendFlightPlan('UAV01',fp2);



%%
monitor.TelemetryViewer('UAV01');



% operator.RemoveUAV('UAV01');

