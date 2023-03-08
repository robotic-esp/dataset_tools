function T_new = orthonormalize_svd(T)
    C = T(1:3,1:3);
    [U, ~, V] = svd(C);
    C_new = U*V';
    T_new = T;
    T_new(1:3,1:3) = C_new;
end