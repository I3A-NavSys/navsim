clear
clc

run('../tools/UTRAFMAN_PATH');



RafaOperator = SimpleOperator("RafaOperator");
% RafaOperator.AirSpace_Test;

for i=0:9
    for j = 0:9
        RafaOperator.AirSpace_DeployModel('base', ...
            ['BASE',num2str(i),num2str(j)], ...
            [i-4.5 j-4.5 0.26],[0 0 0]);
        RafaOperator.AirSpace_DeployModel('drone', ...
            ['UAV',num2str(i),num2str(j)], ...
            [i-4.5 j-4.5 1],[0 0 rand*2+pi]);
    end
end

% 