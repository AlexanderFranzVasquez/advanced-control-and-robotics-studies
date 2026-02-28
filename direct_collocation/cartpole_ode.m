function dx = cartpole_ode(t,x,u_fun,m1,m2,g,l)

    q1 = x(1);   % posición carrito
    q2 = x(2);   % ángulo
    q1d = x(3);  % velocidad carrito
    q2d = x(4);  % velocidad angular

    u = u_fun(t);

    den = m1 + m2*(1 - cos(q2)^2);

    q1dd = ( l*m2*sin(q2)*q2d^2 ...
             + u ...
             + m2*g*cos(q2)*sin(q2) ) / den;

    q2dd = -( l*m2*cos(q2)*sin(q2)*q2d^2 ...
              + u*cos(q2) ...
              + (m1+m2)*g*sin(q2) ) / (l*den);

    dx = [q1d;
          q2d;
          q1dd;
          q2dd];
end