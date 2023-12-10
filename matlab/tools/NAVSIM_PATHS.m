% This file allows you to work with different machines.

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
        otherwise
            error('Windows user not defined in file NAVSIM_init.m');
    end

end

NAVSIM_MODELS_PATH = fullfile(NAVSIM_PATH,'ws/src/navsim_pkg/models/');

addpath(genpath(fullfile(NAVSIM_PATH,'matlab/')));
addpath(fullfile(NAVSIM_PATH,'ws/src/matlab_msg_gen/',PLATFORM,'/install/m/'));
clear PLATFORM 