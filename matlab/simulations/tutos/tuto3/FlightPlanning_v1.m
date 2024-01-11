clear; 
clc;

run('../../../tools/NAVSIM_PATHS');


builder  = SimpleBuilder ('builder' ,NAVSIM_MODELS_PATH);
builder.DeployModel('DC/base_drone','BASE1',[-190 -119 48.25],[0 0 0]);
builder.DeployModel('DC/base_drone','BASE2',[-152 -106 49.25],[0 0 0]);
% builder.DeployModel('DC/base_drone','BASE3',[180 33 50.25],[0 0 0]);


operator = USpaceOperator('operator',NAVSIM_MODELS_PATH);
operator.DeployUAV(UAVmodels.MiniDroneFP1,'UAV01',[-190 -119 48.6],[0 0 0]);




%%
% [s,~] = operator.GetTime();

fp2 = [ 
         -190 -119 48.6   0 0 0   20
%         -190 -119 52.0   0 0 0   40
         -152 -106 52.0   0 0 0   30
         -152 -106 49.6   0 0 0   40
      ];

operator.SendFlightPlan('UAV01',fp2);

