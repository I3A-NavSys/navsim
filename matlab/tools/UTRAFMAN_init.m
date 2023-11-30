% This file allows you to configure ROS master IP address. 
% It is useful if you work with different machines.



% MATLAB PATH configuration
% global UTRAFMAN_DIR

if isunix  %Unix computer

    PLATFORM = 'glnxa64';
    UTRAFMAN_DIR = '~/code/utrafman_ros2/'; %Set it with your repo installation path

elseif ispc %Windows computer

    PLATFORM = 'win64';
    switch getenv("USERNAME")
        case 'Rafael.Casado'
            UTRAFMAN_DIR = 'c:\Users\Rafael.Casado\OneDrive - Universidad de Castilla-La Mancha\NavSys\code\utrafman_ros2\';
        case 'Rafa'
            UTRAFMAN_DIR = 'c:\Users\Rafa\OneDrive - Universidad de Castilla-La Mancha\NavSys\code\utrafman_ros2\';
        otherwise
            error('Windows user not defined in file UTRAFMAN_init.m');
    end

end


addpath(genpath(fullfile(UTRAFMAN_DIR,'matlab/')));
addpath(fullfile(UTRAFMAN_DIR,'utrafman_ws/src/matlab_msg_gen/',PLATFORM,'/install/m/'));

