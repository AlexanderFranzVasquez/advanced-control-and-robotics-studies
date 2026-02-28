function p = foot_placement(L_minus, L_des, params)

    m = params.m;
    H = params.H;
    g = params.g;
    T = params.T;
    alpha = params.alpha;

    ell = sqrt(g/H);

    denom = m*H*ell*sinh(ell*T);

    p = (1 - alpha)/denom * L_des ...
        + (alpha - cosh(ell*T))/denom * L_minus;
end