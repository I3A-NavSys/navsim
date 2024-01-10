clear; 
clc;

run('../../../tools/NAVSIM_PATHS');


builder  = SimpleBuilder ('builder' ,NAVSIM_MODELS_PATH);
builder.DeployModel('DC/base_drone','BASE1',[-190 -120 48.25],[0 0 0]);
builder.DeployModel('DC/base_drone','BASE2',[180 33 50.25],[0 0 0]);



operator = USpaceOperator('operator',NAVSIM_MODELS_PATH);
operator.DeployUAV(UAVmodels.MiniDroneFP1,'UAV01',[-190 -120 48.6],[0 0 0]);



fp1 = [ 
         0 0 0  0 0 0  10
        10 0 1  0 0 0  20
        20 0 2  0 0 0  30
        30 0 3  0 0 0  40
      ];


%%
operator.SendFlightPlan('UAV01',fp1);

