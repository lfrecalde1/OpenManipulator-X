%% Clean variables
clc, clear all, close all;

%% Add Subfolder
addpath("Communication_Code");

%% Define robot
robot = Robot();

%% Position control mode
robot.writeMode('p');
robot.writeMotorState(true);

%% Timing for position interpolation and loop
t_sample = 0.04;   % travel time per commanded point [s]
tfin = 15;
t = 0:t_sample:tfin;

%% Unit factors
factor_degre_to_rad = pi/180;
factor_rad_to_degre = 180/pi;
factor_mA_to_A = 1/1000;

%% Multi-harmonic excitation trajectories (If you are planning to modify this please be careful with collisions and the integrity of the robot)
q1_desired = 1.00*sin(1.0*t) ...
             + 0.60*sin(2.6*t + 0.3) ...
             + 0.45*sin(4.5*t + 1.1) ...
             + 0.30*sin(6.8*t + 0.2) ...
             + 0.20*sin(9.0*t + 1.7);
q2_desired = 0.55*cos(0.6*t + 0.2) + 0.30*sin(1.9*t) + 0.15*cos(3.2*t + 0.7);

q3_desired = -1.0 + 0.45*cos(0.7*t) + 0.25*sin(1.8*t + 0.3) + 0.12*cos(3.4*t);

q4_desired = 0.90*sin(1.1*t) ...
             + 0.55*cos(2.8*t + 0.5) ...
             + 0.35*sin(4.8*t + 1.0) ...
             + 0.25*cos(6.9*t + 0.2) ...
             + 0.15*sin(9.2*t + 1.4);
%% Matrix of the desired joint angles
q_desired_rad = [1*q1_desired; 0.6*q2_desired; 1.0*q3_desired; 0.7*q4_desired];

%% In degres
q_desired_deg = q_desired_rad*factor_rad_to_degre;

%% Data storage
q_real = zeros(4, length(t)+1);
q_dot_real = zeros(4, length(t)+1);
current_real = zeros(4, length(t)+1);
dt = zeros(1, length(t));

%% Read initial condition
joint_readings = robot.getJointsReadings();
q_real(:, 1) = joint_readings(1, :)*factor_degre_to_rad;
q_dot_real(:, 1) = joint_readings(2, :)*factor_degre_to_rad;
current_real(:, 1) = joint_readings(3, :)*factor_mA_to_A;

%% Timed position command loop
for k = 1:length(t)
    tic
    %% Defines the time that the robot has to reach the desired joint
    robot.writeTime(t_sample);

    %% Defines the desired joint
    robot.writeJoints(q_desired_deg(:, k)');

    %% Wait until this sample is completed
    while toc < t_sample
    end
    dt(k) = toc;

    %% Read robot state after reaching commanded sample
    joint_readings = robot.getJointsReadings();
    q_real(:, k+1) = joint_readings(1, :)*factor_degre_to_rad;
    q_dot_real(:, k+1) = joint_readings(2, :)*factor_degre_to_rad;
    current_real(:, k+1) = joint_readings(3, :)*factor_mA_to_A;
end

%% Save data at sample k
q_real_k = q_real(:, 1:length(t));
q_dot_real_k = q_dot_real(:, 1:length(t));
current_real_k = current_real(:, 1:length(t));

save('position_timed_control_data.mat', ...
    't', ...
    'dt', ...
    'q_desired_rad', ...
    'q_real_k', ...
    'q_dot_real_k', ...
    'current_real_k');

save('Identification/position_timed_control_data.mat', ...
    't', ...
    'dt', ...
    'q_desired_rad', ...
    'q_real_k', ...
    'q_dot_real_k', ...
    'current_real_k');

%% Plot desired vs measured joint angles
figure(1)
for i = 1:4
    subplot(2,2,i)
    plot(t, q_desired_rad(i,:), 'r.', 'MarkerSize', 10)
    hold on
    plot(t, q_real_k(i,:), 'b.', 'MarkerSize', 10)
    xlabel('Time [s]')
    ylabel(['q', num2str(i), ' [rad]'])
    title(['Joint ', num2str(i), ': Desired vs Measured'])
    legend('Desired', 'Measured', 'Location', 'best')
    grid on
end

%% End with home command
robot.writeTime(1.0);
robot.writeJoints([0, 0, 0, 0]);
pause(1.0);
disp("Movement Complete")
