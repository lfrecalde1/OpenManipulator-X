function [x_opt_vec, info] = estimate_system_parameters_casadi(q, q_dot, q_ddot, tau, x0,  opts)
% Robust CasADi-based parameter estimation for manipulator dynamics.
% Decision vars:
% x = [m1 m2 m3 m4 ...
%      Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 Ixx4 Iyy4 Izz4 ...
%      fv1 fv2 fv3 fv4]'

%% You will need to modity this path in case you want to identify the system dynamics again
addpath('/home/fer/casadi-3.6.7-linux64-matlab2018b');
import casadi.*;

q = double(q);
q_dot = double(q_dot);
q_ddot = double(q_ddot);
tau = double(tau);

% Fixed geometric parameters
a0 = 0.012;
d1 = 0.0595;
a1 = 0.024;
d2 = 0.128;
a2 = 0.124;
ell_e = 0.126;
g = 9.8;

% Decision variable and bounds
nx = 20;
x = SX.sym('x', nx, 1);
x0 = double(reshape(x0, [], 1));

lb = [1e-3*ones(4,1); 1e-6*ones(12,1); zeros(4,1)];
ub = [5.0*ones(4,1);  1.0*ones(12,1); 2.0*ones(4,1)];

%Build robust objective
cost = SX(0);
N = size(q, 2);
for i = 1:N
    p = [a0; d1; a1; d2; a2; ell_e; ...
         x(1); x(2); x(3); x(4); ...
         x(5); x(6); x(7); ...
         x(8); x(9); x(10); ...
         x(11); x(12); x(13); ...
         x(14); x(15); x(16); ...
         g];

    M = M_fun(q(:, i), p);
    C = C_fun(q(:, i), q_dot(:, i), p);
    G = G_fun(q(:, i), p);

    pf = [x(17); x(18); x(19); x(20)];
    tau_viscous = ViscousFriction_fun(q_dot(:, i), pf);

    tau_model = M*q_ddot(:, i) + C*q_dot(:, i) + G + tau_viscous;
    %r = (tau(:, i) - tau_model) ./ tau_scale;
    r = (tau(:, i) - tau_model);
    % for j = 1:4
    %     z = r(j)/delta;
    %     cost = cost + delta^2 * (sqrt(1 + z^2) - 1);
    % end
    cost = cost + r'*r;
end

nlp = struct('x', x, 'f', cost);

ipopt_opts = struct;
ipopt_opts.ipopt.print_level = opts.print_level;
ipopt_opts.ipopt.max_iter = opts.max_iter;
ipopt_opts.ipopt.tol = 1e-8;
ipopt_opts.ipopt.acceptable_tol = 1e-6;
ipopt_opts.ipopt.linear_solver = 'mumps';
ipopt_opts.print_time = 0;

solver = nlpsol('solver', 'ipopt', nlp, ipopt_opts);
sol = solver('x0', x0, 'lbx', lb, 'ubx', ub);

x_opt_vec = full(sol.x);

%% This is the solver solution and parameters
info = struct();
info.f_opt = full(sol.f);
info.g = opts.g;
info.status = solver.stats();
end
