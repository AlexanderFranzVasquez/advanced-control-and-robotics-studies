function [C Ceq] = nonlcon(x)

% x tiene lo siguiente: u(0) u(1) ... u(9) z1(0) ... z1(9) z2(0) ... z2(9)
% ... z4(0) ... z4(9)
    T = 2; % s
    N = 100; % N muestras
    deltaT = T/(N-1);

    d = 1;    % m
    m1 = 1;   % kg
    m2 = 0.3; % kg
    g = 9.81; % m/s^2
    l = 0.5; % m

    % a = @(z2,z4,u) (l*m2*sin(z2)*z4^2+u*m2*g*cos(z2)*sin(z2))/(m1+m2*(1-cos(z2)^2));
    % b = @(z2,z4,u) -(l*m2*cos(z2)*sin(z2)*z4^2+u*cos(z2)*(m1+m2)*g*sin(z2))/(l*m1+l*m2*(1-cos(z2)^2));

    den = @(th) (m1 + m2*(1 - cos(th).^2));

    a = @(th, thd, u) ...
        ( l*m2*sin(th).*thd.^2 ...
          + u ...
          + m2*g*cos(th).*sin(th) ) ./ den(th);
    
    b = @(th, thd, u) ...
        -( l*m2*cos(th).*sin(th).*thd.^2 ...
           + u.*cos(th) ...
           + (m1+m2)*g*sin(th) ) ...
         ./ ( l*den(th) );

    % Implementacion de la dinamica
    % Le corresponde a z(0) ir desde x((N+1):2*N)
    u  = x((0*N+1):1*N);
    z1 = x((1*N+1):2*N);
    z2 = x((2*N+1):3*N);
    z3 = x((3*N+1):4*N);
    z4 = x((4*N+1):5*N);

    %  implemento par k que va de 1 hasta 9
    % z(k+1) - z1(k) = (deltaT/2) * (z1(k+1) + z1(k))
    % 0 = z1(k) - z(k+1) + (deltaT/2) * (z1(k+1) + z1(k))
    % Implementacion de la dinamica
    % preallocation
    rz1 = zeros(N-1,1);
    rz2 = zeros(N-1,1);
    rz3 = zeros(N-1,1);
    rz4 = zeros(N-1,1);

    for k = 1:(N-1)
        rz1(k) = z1(k) - z1(k+1) + (deltaT/2)*(z3(k+1) + z3(k)); % for k=i
        rz2(k) = z2(k) - z2(k+1) + (deltaT/2)*(z4(k+1) + z4(k)); % for k=i


        % Aqui entra la dinamica no lineal del pendulo y carrito
        a1 = a(z2(k+1),z4(k+1),u(k+1));
        a0 = a(z2(k),z4(k),u(k));

        b1 = b(z2(k+1),z4(k+1),u(k+1));
        b0 = b(z2(k),z4(k),u(k));
      
        rz3(k) = z3(k) - z3(k+1) + (deltaT/2)*(a1+a0); % for k=i necesito calcular a y b
        rz4(k) = z4(k) - z4(k+1) + (deltaT/2)*(b1+b0); % for k=i
    end
    
    % Defino los constrains

    C = [];
    Ceq = [rz1;
           rz2;
           rz3;
           rz4];

    % Defino los constrains de las condiciones finales
    % z1(N) = d;  ->  z1(N)-d = 0
    % z2(N) = pi; ->  z2(N)-pi = 0
    % z3(N) = 0;  ->  z3(N) = 0
    % z4(N) = 0;  ->  z4(N) = 0

    % Y se debe de agregar las condiciones iniciales
    Ceq = [Ceq;
           u(N);
           z1(1);
           z2(1);
           z3(1);
           z4(1);
           z1(N)-d;
           z2(N)-pi;
           z3(N);
           z4(N)];
end