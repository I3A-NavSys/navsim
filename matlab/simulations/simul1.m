clear
clc

run('../tools/UTRAFMAN_init');



RafaOperator = SimpleOperator("RafaOperator");
% RafaOperator.AirSpace_Test;

for i=0:9
    for j = 0:9
        name = ['drone',num2str(i),num2str(j)]
        RafaOperator.AirSpace_DeployUAV(name,i-4.5,j-4.5,10,1);
        % pause(0.1);
    end
end

% 