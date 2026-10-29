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
qFull = homeConfiguration(robot)
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
% syms q1 q2 q3 q4 real
% q_sym = [q1,q2,q3,q4];
% x_sym = openManipulatorPosition(q_sym);
% J_sym = jacobian(x_sym, q_sym);
% matlabFunction(J_sym, 'File', 'getJacobian', 'Vars', {q_sym});
for k = 1:length(t)
    %% Forward Kinematic
    %% Implement this
   
    x = openManipulatorPosition(q);
    

    %% Save data
    qHist(:, k) = q;
    xHist(:, k) = x;
    errHist(:, k) = x_des - x;
    %% Control Law
    %% Implement this
    v_des = [0;0;0];
    qdot = ControLaw(x_des,v_des,x,q, Kp);
    
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

% function [pos_vector] = openManipulatorPosition(joint_vector)
% % pos and joint vector is a 1 x n matrix 
%     q = joint_vector;
%     a0 = 0.012;
%     a1 = 0.024;
%     a2 = 0.124;
%     d1 = 0.0595;
%     d2 = 0.128;
%     le = 0.12;
% 
%     p = [
%         a0 + cos(q(1))*(a1*cos(q(2))+ d2*sin(q(2)) + a2*cos(q(2)+q(3)) + le*cos(q(2)+q(3)+q(4)));
%         sin(q(1))*(a1*cos(q(2)) + d2*sin(q(2)) + a2*cos(q(2)+q(3)) + le*cos(q(2)+q(3)+q(4)));
%         d1 - a1*sin(q(2)) + d2*cos(q(2)) - a2*(sin(q(2)+q(3))) - le*sin(q(2)+q(3)+q(4));
%         ];
%     pos_vector = p;
% end

function [qdot] = ControLaw(desired_pos,desired_velocity,curr_pos,q, Kp)
    J = getJacobian(q');
    e = desired_pos(:) - curr_pos(:) ;
    qdot = pinv(J)*(desired_velocity(:) + Kp*e);

end