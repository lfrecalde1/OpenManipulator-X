%% OpenManipulator-X forward dynamics simulation
clc, clear all, close all;

%% Paths
addpath("Communication_Code");
addpath("generated_dynamics");

%% Timing
t_sample = 0.01;
tfin = 10;
t = 0:t_sample:tfin;
N = length(t);


%% State variables
q = zeros(4, N+1);
q_dot = zeros(4, N+1);
tau_k = zeros(4, N);
dt = zeros(1, N);

%% Desired states for each joint

%% System parameters
R =load('Identification/identification_result.mat');
p = [R.p(1:6); ...
     R.x_opt_vec(1); R.x_opt_vec(2); R.x_opt_vec(3); R.x_opt_vec(4); ...
     R.x_opt_vec(5); R.x_opt_vec(6); R.x_opt_vec(7); ...
     R.x_opt_vec(8); R.x_opt_vec(9); R.x_opt_vec(10); ...
     R.x_opt_vec(11); R.x_opt_vec(12); R.x_opt_vec(13); ...
     R.x_opt_vec(14); R.x_opt_vec(15); R.x_opt_vec(16); ...
     R.id_info.g];
pf = [R.x_opt_vec(17); R.x_opt_vec(18); R.x_opt_vec(19); R.x_opt_vec(20)];

%% Main simulation loop
for k = 1:N
    tic

    % control action at current state your controller should be here
    tau_k(:, k) = 
    while toc < t_sample
    end

    dt(k) = toc;

    % RK4 integration on x = [q; q_dot]
    h = dt(k);
    xk = [q(:,k); q_dot(:,k)];
    
    % This model is just an approximation of the system.
    % Feel free to modify the model based on the system
    f = @(x) [ ...
        x(5:8); ...
        M_fun(x(1:4), p) \ (tau_k(:,k) ...
        - C_fun(x(1:4), x(5:8), p) * x(5:8) ...
        - G_fun(x(1:4), p) ...
        - ViscousFriction_fun(x(5:8), pf)) ...
    ];
    % Runge Kutta
    k1 = f(xk);
    k2 = f(xk + 0.5*h*k1);
    k3 = f(xk + 0.5*h*k2);
    k4 = f(xk + h*k3);

    x_next = xk + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
    % Update the states of the system
    q(:,k+1) = x_next(1:4);
    q_dot(:,k+1) = x_next(5:8);

end