% This file contains the necessary code to compile ROS custom defined messages to be used in MATLAB. 

clc; clear;




if isunix  %Unix computer

    UTRAFMAN_DIR = '~/code/utrafman_ros2/'; %Set it with your repo installation path
    addpath(fullfile(UTRAFMAN_DIR,'utrafman_ws/src/utrafman_pkg/src/matlab_msg_gen_ros1/glnxa64/install/m/'));

elseif ispc %Windows computer

    switch getenv("USERNAME")
        case 'Rafael.Casado'
            UTRAFMAN_DIR = 'c:\Users\Rafael.Casado\OneDrive - Universidad de Castilla-La Mancha\NavSys\code\utrafman_ros2\';
        case 'Rafa'
            UTRAFMAN_DIR = 'c:\Users\Rafa\OneDrive - Universidad de Castilla-La Mancha\NavSys\code\utrafman_ros2\';
        otherwise
            error('Windows user not defined in file UTRAFMAN_init.m');
    end
    addpath(fullfile(UTRAFMAN_DIR,'utrafman_ws\src\utrafman_pkg\src\matlab_msg_gen_ros1\win64\install\m\'));

end

addpath(genpath(fullfile(UTRAFMAN_DIR,'matlab/')));





rosgenmsg(fullfile(UTRAFMAN_DIR,'utrafman_ws/src/utrafman_pkg/src/'));

addpath(fullfile(UTRAFMAN_DIR,'gazebo-ros/src/matlab_msg_gen_ros1/glnxa64/install/m'));

clear classes
rehash toolboxcache
