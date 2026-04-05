%% Add to Path Script

% Absolute path to the libraries on your system
dynamixel_library_path = '/home/fer/PHD_RESEARCH/TA/RBE-502/DynamixelSDK'; % Change this to the absolute location of the dynamixel library on your system
%
% Add necessary folders and subfolders from the Dynamixel Library
addpath(genpath(dynamixel_library_path + "/c/include"));
addpath(dynamixel_library_path + "/c/build/linux64");
addpath(genpath(dynamixel_library_path + "/matlab"));
