clear; clc;
run('../../tools/NAVSIM_PATHS');


builder = SimpleBuilder('builder',NAVSIM_MODELS_PATH);
for i=0:9
    for j = 0:9
        builder.DeployModel('DCmodels/base_drone', ...
            ['BASE',num2str(i),num2str(j)], ...
            [i-4.5 j-4.5 0.26],[0 0 0]);
    end
end

op = DC_Operator("DC_Operator",NAVSIM_MODELS_PATH);
for i=0:9
    for j = 0:9
        op.DeployUAV(['UAV',num2str(i),num2str(j)], ...
                     [i-4.5 j-4.5 1],[0 0 0]); %rand*2*pi
    end
end

op.GetTime()
op.ResetTime;
