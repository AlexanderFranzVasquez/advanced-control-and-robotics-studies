clc; clear; close all

syms theta0 theta1 theta2 theta3
syms d1
syms a2 a3 a4

% alpha a d theta
dh0t1 = [pi/2,  0, d1, theta0];
dh1t2 = [   0, a2,  0, theta1];
dh2t3 = [   0, a3,  0, theta2];
dh3t4 = [pi/2, a4,  0, theta3];

T01 = dh_classic(dh0t1);
T12 = dh_classic(dh1t2);
T23 = dh_classic(dh2t3);
T34 = dh_classic(dh3t4);

T04 = T01*T12*T23*T34;
T04 = simplify(T04);

px = T04(1,4);
py = T04(2,4);
pz = T04(3,4);

d1_val = 10; %cm
a2_val = 50; %cm
a3_val = 30; %cm
a4_val = 10; %cm

params_syms = [d1,a2,a3,a4];
params_vals = [d1_val, a2_val, a3_val, a4_val];

px = subs(px,params_syms,params_vals);
py = subs(py,params_syms,params_vals);
pz = subs(pz,params_syms,params_vals);

syms x y z

f1 = px - x;
f2 = py - y;
f3 = pz - z;

% Definicion de funcion de costo
F = f1^2 + f2^2 + f3^2;
F = simplify(F);

%% Pasas lo symbolic a f_handle

F_handle = matlabFunction(F, 'Vars', {[theta0, theta1, theta2, theta3], x, y, z});
% Esto crea una funcion equivalente F_handle = @(theta, x, y, z) ...

x_des = 30;
y_des = 30;
z_des = 60;

th0 = [0 0 0 0]; % guess inicial

cost_fun = @(theta) F_handle(theta, x_des, y_des, z_des);

theta_opt = fminunc(cost_fun, th0)

double(subs(px, [theta0 theta1 theta2 theta3], theta_opt))
double(subs(py, [theta0 theta1 theta2 theta3], theta_opt))
double(subs(pz, [theta0 theta1 theta2 theta3], theta_opt))

%% Evaluacion preliminar de un punto
figure
P = fk_points([0,0,0,0], params_vals);
plot3(P(1,:), P(2,:), P(3,:), 'o-', ...
          'LineWidth', 2, 'MarkerSize', 6) 
xlabel('X [cm]')
ylabel('Y [cm]')
zlabel('Z [cm]')

% Es necesario poner esto, porque hay un pequeño error computacinal que si
% no se coloca limites en y, parece otro brazo
xlim([-20 90])
ylim([-1 1])
zlim([-10 90])
%% Animacion 3D

t = linspace(0,1,50);
x_traj = 30 + 10*t;
y_traj = 0*ones(size(t));
z_traj = 40 + 25*t; % aqui puede ir variando y viendo graficamente

theta_prev = [0 0 0 0];

% para guardar informacion de las iteraciones
flags = zeros(1, length(t));   
fvals = zeros(1, length(t));   
thetas = zeros(length(t), 4); 

figure
axis equal
grid on
view(3)
hold on

for k = 1:length(t)
    tic
    x_des = x_traj(k);
    y_des = y_traj(k);
    z_des = z_traj(k);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    cost_fun = @(theta) F_handle(theta, x_des, y_des, z_des);
    [theta_opt,fval,flag] = fminunc(cost_fun, theta_prev);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    toc
    
    P = fk_points(theta_opt, params_vals);




    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    flags(k)  = flag;
    fvals(k)  = fval;
    thetas(k,:) = theta_opt;
    

    cla
    plot3(P(1,:), P(2,:), P(3,:), 'o-', ...
          'LineWidth', 2, 'MarkerSize', 6)
    plot3(x_traj, y_traj, z_traj, 'r--')  % trayectoria deseada
    
    xlabel('X [cm]')
    ylabel('Y [cm]')
    zlabel('Z [cm]')

    theta_prev = theta_opt;

    xlim([-20 80])
    ylim([-30 30])
    zlim([0 80])

    drawnow
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
