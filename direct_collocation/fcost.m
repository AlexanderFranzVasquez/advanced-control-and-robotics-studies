function J = fcost(x)

% x tiene lo siguiente: u(0) u(1) ... u(9) z1(0) ... z1(9) z2(0) ... z2(9)
% ... z4(0) ... z4(9)

    T = 2; % s
    N = 100; % N muestras
    deltaT = T/(N-1);

    u  = x((0*N+1):1*N);
    
    % matematicamente: J = deltaT*(u(0)/2 + u(1) + .... + u(8) +u(9)/2)
    sumU = sum((u(2:N-1)).^2) + 0.5*(u(1))^2 + 0.5*(u(N)).^2;
    J = deltaT*sumU;
end