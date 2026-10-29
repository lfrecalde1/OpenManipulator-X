function [pos_vector] = openManipulatorPosition(joint_vector)
% pos and joint vector is a 1 x n matrix 
    q = joint_vector;
    a0 = 0.012;
    a1 = 0.024;
    a2 = 0.124;
    d1 = 0.0595;
    d2 = 0.128;
    le = 0.12;

    p = [
        a0 + cos(q(1))*(a1*cos(q(2))+ d2*sin(q(2)) + a2*cos(q(2)+q(3)) + le*cos(q(2)+q(3)+q(4)));
        sin(q(1))*(a1*cos(q(2)) + d2*sin(q(2)) + a2*cos(q(2)+q(3)) + le*cos(q(2)+q(3)+q(4)));
        d1 - a1*sin(q(2)) + d2*cos(q(2)) - a2*(sin(q(2)+q(3))) - le*sin(q(2)+q(3)+q(4));
        ];
    pos_vector = p;
end