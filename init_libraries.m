%% init_libraries.m
dynamixel_library_path = '/home/tpetry/Documents/WPI/rbe502/OpenManipulator-X-Group2/DynamixelSDK';

if ~isfolder(dynamixel_library_path)
    error('Dynamixel SDK not found at: %s', dynamixel_library_path);
end

include_dir = fullfile(dynamixel_library_path, 'c', 'include');
build_dir   = fullfile(dynamixel_library_path, 'c', 'build', 'linux64');
matlab_dir  = fullfile(dynamixel_library_path, 'matlab');

addpath(genpath(include_dir));
addpath(build_dir);
addpath(genpath(matlab_dir));

% Add the repo itself (Identification/, example scripts, etc.)
addpath(genpath(fileparts(mfilename('fullpath'))));