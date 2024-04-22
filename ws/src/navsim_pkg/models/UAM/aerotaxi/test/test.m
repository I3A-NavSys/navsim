clear; 
clc;
run('../../../../../../../matlab/tools/NAVSIM_PATHS');


operator = USpaceOperator("operator",NAVSIM_MODELS_PATH);

info = UAVinfo('',UAVmodels.MiniDroneCommanded);
info.velMax = 10;
operator.DeployUAV(info,'abejorro',[0 6 1],[0 0 -3]);

info2 = UAVinfo('',UAVmodels.AeroTaxiCommanded);
info2.velMax = 120;
operator.DeployUAV(info2,'aguila',[0 -3 5],[0 0 3]);

pause(5)

%%
operator.RemoteCommand('abejorro',true,0,0,1,0,2);
operator.RemoteCommand('aguila', true,0,0,1,0,2);


