clear; 
clc;

run('../../../tools/NAVSIM_PATHS');


builder  = SimpleBuilder ("builder" ,NAVSIM_MODELS_PATH);
operator = USpaceOperator("operator",NAVSIM_MODELS_PATH);

max = 9;

for i=0:max
    for j = 0:max
        builder.DeployModel('DC/base_drone', ...
            ['BASE',num2str(i),num2str(j)], ...
            [i-4.5 j-4.5 0.26],[0 0 0]);
    end
end


for i=0:max
    for j = 0:max
        operator.DeployUAV(                ...
            UAVmodels.MiniDroneCommanded,  ...
            ['UAV',num2str(i),num2str(j)], ...
            [i-4.5 j-4.5 1],[0 0 0]); %rand*2*pi
    end
end

pause(2)

for i=0:max
    for j = 0:max
        operator.RemoteCommand( ...
            ['UAV',num2str(i),num2str(j)], ...
            true,0,0,0.5,0,1);
    end
end

pause(10)

for i=0:max
    for j = 0:max
        operator.RemoteCommand( ...
            ['UAV',num2str(i),num2str(j)], ...
            true,2,0,0,1,100);
    end
end

pause(30)


for i=0:max
    for j = 0:max
        operator.RemoteCommand( ...
            ['UAV',num2str(i),num2str(j)], ...
            false,0,0,0,0,1);
    end
end

