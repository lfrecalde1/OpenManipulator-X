function [gradw] = openManipulatorwGradient(joint_vector)
    
% pos and joint vector is a matrix 
    syms q1 q2 q3 q4;
    q = [q1; q2; q3; q4];

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

    jacob_sym = jacobian([p(1), p(2), p(3)], [q1, q2, q3, q4]);
    w = sqrt(det(jacob_sym*jacob_sym'));
    gradw_sym = gradient(w, q);

    
    gradw = double(subs(gradw_sym, q, joint_vector(:)));

end