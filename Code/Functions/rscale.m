function Nbar = rscale(sys, K)
    [n, ~] = size(sys.A);
    Z = [sys.A, sys.B; sys.C, sys.D];
    % Use pseudo-inverse for possibly non-square matrix
    N = pinv(Z) * [zeros(n,1); 1];
    Nx = N(1:n);
    Nu = N(n+1:end);
    Nbar = Nu + K * Nx;
end