% This file contains the necessary code to compile ROS custom defined messages to be used in MATLAB. 

clc; clear;


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


ros2genmsg(fullfile(UTRAFMAN_DIR,'utrafman_ws/src/utrafman_pkg/interfaces'));
addpath(fullfile(UTRAFMAN_DIR,'utrafman_ws/src/utrafman_pkg/interfaces/matlab_msg_gen/',PLATFORM,'/install/m/'));

clear classes
rehash toolboxcache
