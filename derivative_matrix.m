function Mp = derivative_matrix(M, qp, q)
% Correct time derivative Mp(i,j) = d/dt(M(i,j))
n = size(M, 1);
m = size(M, 2);
Mp = sym(zeros(n, m));
for i = 1:n
    for j = 1:m
        Mp(i,j) = jacobian(M(i,j), q) * qp;
    end
end
Mp = simplify(Mp);
end

