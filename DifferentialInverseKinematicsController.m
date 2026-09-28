clc, clear all, close all;

%% Load rigid body tree
robot = createRigidBodyTree;
robot.DataFormat = "column";
robot.Gravity = [0; 0; -9.81];

%% Controller setup
eeName = "end_effector";

%% Time Variables
dt = 0.05;
Tfinal = 5;
t = 0:dt:Tfinal;

%% The URDF has 6 movable joints: 4 arm joints + 2 gripper joints.
q = [0; 0; 0; 0];
qFull = homeConfiguration(robot);
qFull(:) = 0;

%% The Joints we can control for position control
armJointIdx = 1:4;

%% Geometric parameters
params.a0 = 0.012;
params.d1 = 0.0595;
params.a1 = 0.024;
params.d2 = 0.128;
params.a2 = 0.124;
params.ell_e = 0.12;

%% Desired Location of the end effector
x_des = [0.20; 0.20; 0.15];    
Kp = 5.0;

qHist = zeros(numel(q), length(t));
qdotHist = zeros(numel(q), length(t));
xHist = zeros(3, length(t));
errHist = zeros(3, length(t));

%% Simulate differential inverse kinematics controller
figure;
for k = 1:length(t)
    %% Forward Kinematic
    %% Implement this
    x = openManipulatorPosition();
    
    %% Save data
    qHist(:, k) = q;
    xHist(:, k) = x;
    errHist(:, k) = x_des - x;
    %% Control Law
    %% Implement this
    qdot = ControLaw();
    
    %% Integration of the velocities
    q = q + dt*qdot;
    
    %% Update joints
    qFull(armJointIdx) = q;
    show(robot, qFull, "PreservePlot", false, "Frames", "off");
    hold on;
    plot3(x_des(1), x_des(2), x_des(3), "ro", "MarkerSize", 8, "LineWidth", 2);
    hold off;
    axis([-0.1 0.4 -0.3 0.3 0 0.4]);
    grid on;
    drawnow;
end

