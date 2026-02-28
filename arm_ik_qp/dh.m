function [T] = dh(dh_params)

alpha = dh_params(1);
a = dh_params(2);
d = dh_params(3);
theta = dh_params(4);

% The transformation matrix T is computed based on the Denavit-Hartenberg parameters.
T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
     sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
     0         ,             sin(alpha),             cos(alpha),            d;
     0         ,                      0,                      0,            1];

end