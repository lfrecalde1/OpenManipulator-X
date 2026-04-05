function C = C_matrix(M, q, qp)
%% Coriolis matrix using Christoffel symbols
n = size(M, 1);
C = sym(zeros(n, n));
for i = 1:n
    for j = 1:n
        cij = sym(0);
        for k = 1:n
            cijk = sym(1/2) * (diff(M(i,j), q(k)) + diff(M(i,k), q(j)) - diff(M(j,k), q(i)));
            cij = cij + cijk * qp(k);
        end
        C(i,j) = simplify(cij);
    end
end
end