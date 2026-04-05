%% OpenManipulator-X symbolic kinematics and dynamics (4-DOF)
clc; clear; close all;

%% Generalized coordinates
syms q1 q2 q3 q4 real
syms q1p q2p q3p q4p real
syms q1pp q2pp q3pp q4pp real
q = [q1; q2; q3; q4];
qp = [q1p; q2p; q3p; q4p];
qpp = [q1pp; q2pp; q3pp; q4pp];

%% Physical parameters
syms m1 m2 m3 m4 g real
syms Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 Ixx4 Iyy4 Izz4 real
syms a0 d1 a1 d2 a2 ell_e real

%% Forward kinematics
T_base = trans_xyz(a0, 0, d1);
A01 = dh_axis(q1, 0,   0,    0, 'z');
A12 = dh_axis(q2, d2,  a1,   0, 'y');
A23 = dh_axis(q3, 0,   a2,   0, 'y');
A34 = dh_axis(q4, 0, ell_e,  0, 'y');

%% Use simplify to get a better symbolic representation
T01 = simplify(T_base*A01);
T02 = simplify(T01*A12);
T03 = simplify(T02*A23);
T04 = simplify(T03*A34);

%% End Effector position Vector
p_ee = simplify(T04(1:3,4))

%% Link CoM positions
T0c1 = simplify(T01*dh_axis(q2, d2/2, a1/2, 0, 'y'));
T0c2 = simplify(T02*dh_axis(q3, 0,    a2/2, 0, 'y'));
T0c3 = simplify(T03*dh_axis(q4, 0,  ell_e/2, 0, 'y'));
T0c4 = T04;

%% Simplify symbolic expressions
p_c1 = simplify(T0c1(1:3,4));
p_c2 = simplify(T0c2(1:3,4));
p_c3 = simplify(T0c3(1:3,4));
p_c4 = simplify(T0c4(1:3,4));

links = cat(3, p_c1, p_c2, p_c3, p_c4);
mass_total = [m1; m2; m3; m4];

%% Translational Jacobians
Jv1 = jacobian(p_c1, q);
Jv2 = jacobian(p_c2, q);
Jv3 = jacobian(p_c3, q);
Jv4 = jacobian(p_c4, q);
J_t = cat(3, Jv1, Jv2, Jv3, Jv4);

%% Rotational Jacobians
R01 = T01(1:3,1:3);
R02 = T02(1:3,1:3);
R03 = T03(1:3,1:3);

z0 = [0;0;1];
y1 = simplify(R01*[0;1;0]);
y2 = simplify(R02*[0;1;0]);
y3 = simplify(R03*[0;1;0]);

Jw1 = [z0, y1, sym(zeros(3,1)), sym(zeros(3,1))];
Jw2 = [z0, y1, y2,               sym(zeros(3,1))];
Jw3 = [z0, y1, y2,               y3];
Jw4 = [z0, y1, y2,               y3];
J_r = cat(3, Jw1, Jw2, Jw3, Jw4);

%% Inertia and rotation matrices per link
I1 = diag([Ixx1, Iyy1, Izz1]);
I2 = diag([Ixx2, Iyy2, Izz2]);
I3 = diag([Ixx3, Iyy3, Izz3]);
I4 = diag([Ixx4, Iyy4, Izz4]);
inertia_total = cat(3, I1, I2, I3, I4);

R0c1 = T0c1(1:3,1:3);
R0c2 = T0c2(1:3,1:3);
R0c3 = T0c3(1:3,1:3);
R0c4 = T0c4(1:3,1:3);
rotationals = cat(3, R0c1, R0c2, R0c3, R0c4);

%% Dynamics
M_full = simplify(M_matrix(J_t, J_r, mass_total, inertia_total, rotationals, q));
C_full = simplify(C_matrix(M_full, q, qp));
M_p = simplify(derivative_matrix(M_full, qp, q));

%% Validation of the symbolic dynamics of the Manipulator
Difference = simplify(M_p - 2 * C_full);
skew_residual = simplify(Difference + Difference.');

%% If the dynamics are correct this shoudl be zero Robotics Modelling, Planning and Control 
energy_residual = simplify(qp.' * Difference * qp);

%% Compute gravity Vector
g_vec = [0; 0; g];
G_full = simplify(G_matrix(links, mass_total, q, g_vec))

%% Friction model
syms fv1 fv2 fv3 fv4 real
Fv = diag([fv1, fv2, fv3, fv4]);
tau_viscous = simplify(Fv*qp);

% Full inverse-dynamics torque with friction
tau_model_no_friction = simplify(M_full*qpp + C_full*qp + G_full);

disp('Computed symbolic dynamics:')
disp('M_full size ='); disp(size(M_full))
disp('C_full size ='); disp(size(C_full))
disp('G_full size ='); disp(size(G_full))
disp('Fv size ='); disp(size(Fv))
disp('skew_residual = (Mdot-2C) + (Mdot-2C)'' :')
disp(skew_residual)
disp('energy_residual = qdot''*(Mdot-2C)*qdot :')
disp(energy_residual)

%% Auto-generate numeric functions (matlabFunction)
% Parameter vector order:
% p = [a0 d1 a1 d2 a2 ell_e m1 m2 m3 m4 Ixx1 Iyy1 Izz1 Ixx2 Iyy2 Izz2 Ixx3 Iyy3 Izz3 Ixx4 Iyy4 Izz4 g]'
p = [a0; d1; a1; d2; a2; ell_e; ...
     m1; m2; m3; m4; ...
     Ixx1; Iyy1; Izz1; ...
     Ixx2; Iyy2; Izz2; ...
     Ixx3; Iyy3; Izz3; ...
     Ixx4; Iyy4; Izz4; ...
     g];

if ~exist('generated_dynamics', 'dir')
    mkdir('generated_dynamics');
end

if ~exist('Identification/generated_dynamics', 'dir')
    mkdir('Identification/generated_dynamics');
end

matlabFunction(M_full, 'File', 'generated_dynamics/M_fun', 'Vars', {q, p}, 'Optimize', true);
matlabFunction(C_full, 'File', 'generated_dynamics/C_fun', 'Vars', {q, qp, p}, 'Optimize', true);
matlabFunction(G_full, 'File', 'generated_dynamics/G_fun', 'Vars', {q, p}, 'Optimize', true);
matlabFunction(p_ee,   'File', 'generated_dynamics/FK_fun', 'Vars', {q, p}, 'Optimize', true);

matlabFunction(M_full, 'File', 'Identification/M_fun', 'Vars', {q, p}, 'Optimize', true);
matlabFunction(C_full, 'File', 'Identification/C_fun', 'Vars', {q, qp, p}, 'Optimize', true);
matlabFunction(G_full, 'File', 'Identification/G_fun', 'Vars', {q, p}, 'Optimize', true);
matlabFunction(p_ee,   'File', 'Identification/FK_fun', 'Vars', {q, p}, 'Optimize', true);

% Friction parameter vector order:
pf = [fv1; fv2; fv3; fv4];
matlabFunction(tau_viscous, ...
    'File', 'generated_dynamics/ViscousFriction_fun', ...
    'Vars', {qp, pf}, 'Optimize', true);

matlabFunction(tau_viscous, ...
    'File', 'Identification/ViscousFriction_fun', ...
    'Vars', {qp, pf}, 'Optimize', true);

disp('Generated functions:')
disp(' - generated_dynamics/M_fun.m')
disp(' - generated_dynamics/C_fun.m')
disp(' - generated_dynamics/G_fun.m')
disp(' - generated_dynamics/FK_fun.m')
disp(' - generated_dynamics/ViscousFriction_fun.m')

disp('Generated functions:')
disp(' - Identification/M_fun.m')
disp(' - Identification/C_fun.m')
disp(' - Identification/G_fun.m')
disp(' - Identification/FK_fun.m')
disp(' - Identification/ViscousFriction_fun.m')




function A = dh_axis(theta, d, a, alpha, axis_name)
    if axis_name == 'z'
        Rtheta = rotz_h(theta);
    elseif axis_name == 'y'
        Rtheta = roty_h(theta);
    else
        error('axis_name must be ''z'' or ''y''.');
    end
    A = Rtheta * trans_xyz(0, 0, d) * trans_xyz(a, 0, 0) * rotx_h(alpha);
end

function T = trans_xyz(x, y, z)
    T = [1 0 0 x;
         0 1 0 y;
         0 0 1 z;
         0 0 0 1];
end

function R = rotx_h(a)
    R = [1 0       0      0;
         0 cos(a) -sin(a) 0;
         0 sin(a)  cos(a) 0;
         0 0       0      1];
end

function R = roty_h(a)
    R = [ cos(a) 0 sin(a) 0;
          0      1 0      0;
         -sin(a) 0 cos(a) 0;
          0      0 0      1];
end

function R = rotz_h(a)
    R = [cos(a) -sin(a) 0 0;
         sin(a)  cos(a) 0 0;
         0       0      1 0;
         0       0      0 1];
end

