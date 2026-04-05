function tau = current_to_torque(I)
%CURRENT_TO_TORQUE Exact inverse of torque_to_current when If = 0

    k = 0.57;   % A / (N*m)

    tau = I ./ k;
end