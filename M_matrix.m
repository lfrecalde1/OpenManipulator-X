function M = M_matrix(J_t, J_r, mass, inertia, rotationals, q)
%% System dimention
n = size(q, 1);
%% Dimention total positions
l = size(J_t, 3);

%% Aux M Matrix 
syms aux;
M = zeros(n, n)*aux;
for k =1:l
    % Translational jacobinas
    Jt = J_t(:,:,k);
    % Rotational jacobinas
    Jr = J_r(:,:,k);
    % Inertia Matrix
    I = inertia(:,:,k);
    % Rotational Matrix
    R = rotationals(:,:,k);
    
    % Value
    value = (mass(k,1)*(Jt'*Jt))+(Jr'*R*I*R'*Jr);
    M = M +(value);
end
M = simplify(M);
end