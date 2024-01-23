% This file contains the necessary code to compile ROS custom defined messages to be used in MATLAB. 

clc; clear;


if isunix  %Unix computer

    PLATFORM = 'glnxa64';
    NAVSIM_PATH = '~/code/navsim/'; %Set it with your repo installation path

elseif ispc %Windows computer

    PLATFORM = 'win64';
    switch getenv("USERNAME")
        case 'Rafael.Casado'
            NAVSIM_PATH = 'c:\Users\Rafael.Casado\code\navsim\';
        case 'Rafa'
            NAVSIM_PATH = 'c:\Users\Rafa\code\navsim\';
        case 'rafae'
            NAVSIM_PATH = 'c:\Users\rafae\code\navsim\';
        otherwise
            error('NAVSIM_DIR not defined');
    end

end


ros2genmsg(fullfile(NAVSIM_PATH,'ws/src/'));
addpath(fullfile(NAVSIM_PATH,'ws/src/matlab_msg_gen/',PLATFORM,'/install/m/'));

clear classes
rehash toolboxcache
