%% Setup robot
clc, clear all, close all;

%% Add Subfolder
addpath("Communication_Code");

%% Define the time, this is time for a interpolation between where the robot is now and where you want to be
%% For control We do not need it but if you want to play with the position control of each joint try to decrease this time.
%% Do not decrease less than 1 since this can produce agressive movement
travelTime = 1.0; 

%% Create the robot
robot = Robot(); 
%% Define the type of low level controller, this is position control for each joint
robot.writeMode('p');

%% This defines the time for the interpolation.
%% Yous shoudl define this if you are using position control
robot.writeTime(travelTime); 

%% Activate the torque
robot.writeMotorState(true); 

disp("Initializing...")

%% Send zero to each joint this is Home of the robot
robot.writeJoints([0,0,0,0]); 
pause(travelTime); 

%% New Point for joint 1
baseWayPoints = [-30, 30, 0]; 

disp("Executing Movement...")
for baseWayPoint = baseWayPoints 
    robot.writeJoints([baseWayPoint, 0, 0, 0]);

    tic; % Start timer
    while toc < travelTime
        disp(robot.getJointsReadings()); 
    end
end

disp("Movement Complete")
robot.writeGripper(false);
pause(1);