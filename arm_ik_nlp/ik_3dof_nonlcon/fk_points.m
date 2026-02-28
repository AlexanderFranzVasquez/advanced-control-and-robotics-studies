function P = fk_points(theta)
    
    d1 = 10;
    a2 = 50;
    a3 = 20;

    
    % alpha a d theta
    dh0t1 = [pi/2,  0, d1, theta(1)];
    dh1t2 = [   0, a2,  0, theta(2)];
    dh2t3 = [pi/2, a3,  0, theta(3)];
    
    T01 = dh_classic(dh0t1);
    T12 = dh_classic(dh1t2);
    T23 = dh_classic(dh2t3);
    
    T02 = T01*T12;
    T03 = T02*T23;
    
    P0 = [0;0;0];
    P1 = T01(1:3,4);
    P2 = T02(1:3,4);
    P3 = T03(1:3,4);
    
    P = [P0 P1 P2 P3];  % columnas = puntos
end