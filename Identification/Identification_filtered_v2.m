%% Identification with consistent filtering for q, qdot, qddot, tau
clc; clear; close all;

%% Define parameters
a0 = 0.012;
d1 = 0.0595;
a1 = 0.024;
d2 = 0.128;
a2 = 0.124;
ell_e = 0.126;

%% Initial model parameters for generated dynamics functions
m1 = 0.1; m2 = 0.1; m3 = 0.1; m4 = 0.1;
Ixx1 = 0.1; Iyy1 = 0.1; Izz1 = 0.1;
Ixx2 = 0.1; Iyy2 = 0.1; Izz2 = 0.1;
Ixx3 = 0.1; Iyy3 = 0.1; Izz3 = 0.1;
Ixx4 = 0.1; Iyy4 = 0.1; Izz4 = 0.1;
g = 9.8;

p = [a0; d1; a1; d2; a2; ell_e; ...
     m1; m2; m3; m4; ...
     Ixx1; Iyy1; Izz1; ...
     Ixx2; Iyy2; Izz2; ...
     Ixx3; Iyy3; Izz3; ...
     Ixx4; Iyy4; Izz4; ...
     g];

%% Quick function check
q_test = [-0.816316142424989; 0.273022363210459; -0.400934207323283; -1.173956571756373];
qd_test = [-0.1; 0.2; 0.1; -0.5];
x = FK_fun(q_test, p); 
M = M_fun(q_test, p); 
C = C_fun(q_test, qd_test, p); 
G = G_fun(q_test, p); 

%% Load data
load('position_timed_control_data.mat');
q_desired = q_desired_rad;
tau = current_to_torque(current_real_k);

%% Force row-vector time
t = t(:)';
dt = mean(diff(t));
fs = 1/dt;

%% One shared low-pass filter for all signals (zero-phase)
lambda = 50;                   % [rad/s]
fc = lambda/(2*pi);            % [Hz]
Wn = fc/(fs/2);                % normalized cutoff
Wn = min(max(Wn, 1e-4), 0.99); % safe bounds
[b, a] = butter(2, Wn, 'low');

q_real_filter = filtfilt_rows(b, a, q_real_k);
q_real_dot_filter = filtfilt_rows(b, a, q_dot_real_k);
tau_filter = filtfilt_rows(b, a, tau);

% Acceleration from velocity, then same filter again
q_real_ddot_est = gradient_rows(q_real_dot_filter, dt);
q_real_ddot_filter = filtfilt_rows(b, a, q_real_ddot_est);

%% Plots
figure(1)
for i = 1:4
    subplot(2,2,i)
    plot(t, q_desired(i,:), '.r'); hold on
    plot(t, q_real_k(i,:), '.b');
    plot(t, q_real_filter(i,:), '.m');
    xlabel('Time [s]')
    ylabel(['q', num2str(i), ' [rad]'])
    title(['Joint ', num2str(i), ' Angle'])
    legend('Desired', 'Measured', 'Filtered', 'Location', 'best')
    grid on
end

figure(2)
for i = 1:4
    subplot(2,2,i)
    plot(t, q_dot_real_k(i,:), '.k'); hold on
    plot(t, q_real_dot_filter(i,:), '.m');
    xlabel('Time [s]')
    ylabel(['dq', num2str(i), ' [rad/s]'])
    title(['Joint ', num2str(i), ' Velocity'])
    legend('Measured', 'Filtered', 'Location', 'best')
    grid on
end

figure(3)
for i = 1:4
    subplot(2,2,i)
    plot(t, tau(i,:), '.k'); hold on
    plot(t, tau_filter(i,:), '.m');
    xlabel('Time [s]')
    ylabel(['tau', num2str(i), ' [Nm]'])
    title(['Joint ', num2str(i), ' Torque'])
    legend('Measured', 'Filtered', 'Location', 'best')
    grid on
end

figure(4)
for i = 1:4
    subplot(2,2,i)
    plot(t, q_real_ddot_est(i,:), '.c'); hold on
    plot(t, q_real_ddot_filter(i,:), '.m');
    xlabel('Time [s]')
    ylabel(['ddq', num2str(i), ' [rad/s^2]'])
    title(['Joint ', num2str(i), ' Acceleration from Velocity'])
    legend('Estimated', 'Filtered', 'Location', 'best')
    grid on
end

%% Data prepared for identification
q_id = q_real_filter;
qd_id = q_real_dot_filter;
qdd_id = q_real_ddot_filter;
tau_id = tau_filter;

save('identification_signals_filtered.mat', ...
    't', 'q_id', 'qd_id', 'qdd_id', 'tau_id', 'q_desired', 'p', 'lambda', 'fc');


%% System Identification
x0_dyn = [7.9119962e-02; 9.8406837e-02; 1.3850917e-01; 1.3274562e-01; ...
          1.2505234e-05; 2.1898364e-05; 1.9267361e-05; ...
          3.4543422e-05; 3.2689329e-05; 1.8850320e-05; ...
          3.3055381e-04; 3.4290447e-04; 6.0346498e-05; ...
          3.0654178e-05; 2.4230292e-04; 2.5155057e-04];

%% Friction initial guess: [fv1..fv4 fc1..fc4]
x0_fric = 1e-2*ones(4,1);
x0 = [x0_dyn; x0_fric];

%% Some parameters for the optimizer
id_opts = struct();
id_opts.max_iter = 1000;
id_opts.print_level = 5;
id_opts.g = g;

[x_opt_vec, id_info] = estimate_system_parameters_casadi(q_id, qd_id, qdd_id, tau_id, x0, id_opts);

disp('Estimated parameters x_opt_vec =')
disp(x_opt_vec)
disp('Optimization cost =')
disp(id_info.f_opt)

save('identification_result.mat', 'x_opt_vec', 'id_info', 'x0', 'id_opts', 'p')


%% Local helper functions
function y = filtfilt_rows(b, a, x)
    y = zeros(size(x));
    for ii = 1:size(x,1)
        y(ii,:) = filtfilt(b, a, x(ii,:));
    end
end

function xd = gradient_rows(x, dt)
    xd = zeros(size(x));
    for ii = 1:size(x,1)
        xd(ii,:) = gradient(x(ii,:), dt);
    end
end
