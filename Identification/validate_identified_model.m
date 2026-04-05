%% Validate identified dynamics model using torque reconstruction
clc; clear; close all;

S = load('identification_signals_filtered.mat');
t = S.t(:)';
q_id = S.q_id;
qd_id = S.qd_id;
qdd_id = S.qdd_id;
tau_meas = S.tau_id;

%% Load identified parameters
R = load('identification_result.mat');
x_opt_vec = R.x_opt_vec(:);

%% Build full parameter vector p
p = [R.p(1:6); ...
     x_opt_vec(1); x_opt_vec(2); x_opt_vec(3); x_opt_vec(4); ...
     x_opt_vec(5); x_opt_vec(6); x_opt_vec(7); ...
     x_opt_vec(8); x_opt_vec(9); x_opt_vec(10); ...
     x_opt_vec(11); x_opt_vec(12); x_opt_vec(13); ...
     x_opt_vec(14); x_opt_vec(15); x_opt_vec(16); ...
     R.id_info.g];


pf = [x_opt_vec(17); x_opt_vec(18); x_opt_vec(19); x_opt_vec(20)];


N = size(q_id, 2);
tau_hat = zeros(4, N);
tau_visc_all = zeros(4, N);
tau_stat_all = zeros(4, N);

for k = 1:N
    M = M_fun(q_id(:,k), p);
    C = C_fun(q_id(:,k), qd_id(:,k), p);
    G = G_fun(q_id(:,k), p);
    tau_visc = ViscousFriction_fun(qd_id(:,k), pf);
    tau_hat(:,k) = M*qdd_id(:,k) + C*qd_id(:,k) + G + tau_visc;
end

res = tau_meas - tau_hat;

%% Metrics per joint
rmse = zeros(4,1);
nrmse = zeros(4,1);
r2 = zeros(4,1);
bias = zeros(4,1);
for j = 1:4
    y = tau_meas(j,:);
    yhat = tau_hat(j,:);
    e = y - yhat;

    rmse(j) = sqrt(mean(e.^2));
    denom = max(y) - min(y);
    if denom < 1e-8
        denom = std(y);
    end
    if denom < 1e-8
        denom = 1;
    end
    nrmse(j) = rmse(j)/denom;

    sse = sum((y - yhat).^2);
    sst = sum((y - mean(y)).^2);
    if sst < 1e-12
        r2(j) = NaN;
    else
        r2(j) = 1 - sse/sst;
    end
    bias(j) = mean(e);
end

metrics_table = table((1:4)', rmse, nrmse, r2, bias, ...
    'VariableNames', {'Joint', 'RMSE', 'NRMSE', 'R2', 'Bias'});

disp('Validation metrics (measured tau vs reconstructed tau_hat):')
disp(metrics_table)
disp('Validation used friction model: tau = M*qdd + C*qd + G + tau_viscous')


%% Plots: measured vs predicted torque
figure(1)
for j = 1:4
    subplot(2,2,j)
    plot(t, tau_meas(j,:), 'k', 'LineWidth', 1.0); hold on
    plot(t, tau_hat(j,:), 'r--', 'LineWidth', 1.0);
    xlabel('Time [s]')
    ylabel(['tau', num2str(j), ' [Nm]'])
    title(['Joint ', num2str(j), ': Measured vs Predicted'])
    legend('Measured', 'Predicted', 'Location', 'best')
    grid on
end

%% Plots: residuals
figure(2)
for j = 1:4
    subplot(2,2,j)
    plot(t, res(j,:), 'b', 'LineWidth', 1.0); hold on
    yline(0, 'k--');
    xlabel('Time [s]')
    ylabel(['e', num2str(j), ' [Nm]'])
    title(['Joint ', num2str(j), ': Residual'])
    grid on
end
