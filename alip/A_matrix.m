function A = A_matrix(params)

    m = params.m;
    H = params.H;
    g = params.g;
    T = params.T;

    ell = sqrt(g/H);

    A = [ cosh(ell*T),              sinh(ell*T)/(m*H*ell);
          m*H*ell*sinh(ell*T),      cosh(ell*T) ];
end