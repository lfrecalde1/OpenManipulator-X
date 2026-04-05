function I = torque_to_current(tau)
%TORQUE_TO_CURRENT_PIECEWISE Piecewise torque-current model
    k = 0.57;
    If = 0.00;

    I = k .* tau + If .* sign(tau);
end