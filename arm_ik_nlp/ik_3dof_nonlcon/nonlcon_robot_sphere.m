function [c,ceq] = nonlcon_robot_sphere(theta, cObs, R)
    P = fk_points(theta); % 3xM puntos del robot
    % distancia de cada punto al centro
    d = sqrt( (P(1,:)-cObs(1)).^2 + (P(2,:)-cObs(2)).^2 + (P(3,:)-cObs(3)).^2 );
    % imponer d >= R  =>  R - d <= 0 para cada punto
    c = (R - d(:));
    ceq = [];
end