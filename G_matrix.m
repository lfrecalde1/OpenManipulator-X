function G = G_matrix(links,mass,q, g)
%% Dimention total positions
l = size(links, 3);

syms aux;
V = zeros(1, 1)*aux;

for k =1:l
    value = (mass(k,1)*g'*links(:,:,k));
    V = V +(value);
end
V = simplify(V);

G = simplify(jacobian(V,q))';
end