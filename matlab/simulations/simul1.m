clear
clc

run('../tools/UTRAFMAN_init');



RafaOperator = SimpleOperator("RafaOperator");
RafaOperator.AirSpace_Test;

for i=1:10
    RafaOperator.AirSpace_DeployUAV;
end

