clear; 
clc;

run('../../../tools/NAVSIM_PATHS');


builder  = SimpleBuilder ("builder" ,NAVSIM_MODELS_PATH);
builder.DeployModel('DC/base_drone','BASE1',[-190 -120 48.25],[0 0 0]);
builder.DeployModel('DC/base_drone','BASE2',[180 33 50.25],[0 0 0]);



operator = USpaceOperator("operator",NAVSIM_MODELS_PATH);
operator.DeployUAV(UAVmodels.MiniDroneFP1,'UAV',[-190 -120 48.6],[0 0 0]);

