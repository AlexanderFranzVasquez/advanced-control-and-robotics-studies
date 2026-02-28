function [xd] = dynSelfBalancing(t,x,params,k)

    % Unpack parameters
    m = params.m;          
    M = params.M;        
    R = params.R;     
    Jw = params.Jw;  
    l = params.l; 
    Jc = params.Jc;
    u0 = params.u0; 
    u1 = params.u1; 
    g = params.g;

    Ts    = params.Ts;
    sigma = params.sigma;

    % Persistent variables for sample-and-hold noise
    persistent v k_last

    if isempty(v)
        v = zeros(3,1);     % noise for [x2, x3, x4]
        k_last = -1;
    end

    % Current discrete time index
    k_now = floor(t / Ts);

    % Update noise only every Ts seconds
    if k_now ~= k_last
        v = sigma * randn(3,1);
        k_last = k_now;
    end

    a = m + 2*M + 2* Jw/R^2;
    b = m*l;
    c = m*l^2 + Jc;
    delta = a*c-b^2;

    x1 = x(1);
    x2 = x(2);
    x3 = x(3);
    x4 = x(4);

    % Measured states (with noise) 
    x2m = x2 + v(1);
    x3m = x3 + v(2);
    x4m = x4 + v(3);

    u = -k(1)*x2m - k(2)*x3m - k(3)*x4m;

    x1d = x2;
    x2d = (  c*(m*l*sin(x3)*x4^2-2*u0*x2) - b*(u-2*u1*x4+m*g*l*sin(x3)) )/delta;
    x3d = x4;
    x4d = ( -b*(m*l*sin(x3)*x4^2-2*u0*x2) + a*(u-2*u1*x4+m*g*l*sin(x3)) )/delta;


    
    xd = [x1d;
          x2d;
          x3d;
          x4d];
end