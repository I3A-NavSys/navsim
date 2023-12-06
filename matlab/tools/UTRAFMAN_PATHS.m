% This file allows you to work with different machines.

if isunix  %Unix computer

    PLATFORM = 'glnxa64';
    UTRAFMAN_PATH = '~/code/utrafman_ros2/'; %Set it with your repo installation path

elseif ispc %Windows computer

    PLATFORM = 'win64';
    switch getenv("USERNAME")
        case 'Rafael.Casado'
            UTRAFMAN_PATH = 'c:\Users\Rafael.Casado\code\utrafman_ros2\';
        case 'Rafa'
            UTRAFMAN_PATH = 'c:\Users\Rafa\code\utrafman_ros2\';
        otherwise
            error('Windows user not defined in file UTRAFMAN_init.m');
    end

end

UTRAFMAN_MODELS_PATH = fullfile(UTRAFMAN_PATH,'utrafman_ws/src/utrafman_pkg/models/');

addpath(genpath(fullfile(UTRAFMAN_PATH,'matlab/')));
addpath(fullfile(UTRAFMAN_PATH,'utrafman_ws/src/matlab_msg_gen/',PLATFORM,'/install/m/'));
clear PLATFORM 