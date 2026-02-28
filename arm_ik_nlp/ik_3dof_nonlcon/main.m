clc; clear; close all

syms theta0 theta1 theta2 
syms d1
syms a2 a3

% alpha a d theta
dh0t1 = [pi/2,  0, d1, theta0];
dh1t2 = [   0, a2,  0, theta1];
dh2t3 = [pi/2, a3,  0, theta2];

T01 = dh_classic(dh0t1);
T12 = dh_classic(dh1t2);
T23 = dh_classic(dh2t3);

T03 = T01*T12*T23;
T03 = simplify(T03);

px = T03(1,4);
py = T03(2,4);
pz = T03(3,4);

d1_val = 10; %cm
a2_val = 50; %cm
a3_val = 20; %cm

params_syms = [d1,a2,a3];
params_vals = [d1_val, a2_val, a3_val];

px = subs(px,params_syms,params_vals);
py = subs(py,params_syms,params_vals);
pz = subs(pz,params_syms,params_vals);

%% DESDE AQUI PARTIERON EN EL LAB
syms x y z

f1 = px - x;
f2 = py - y;
f3 = pz - z;

% Definicion de funcion de costo
F = f1^2 + f2^2 + f3^2;
F = simplify(F);

%% Pasas lo symbolic a f_handle

F_handle = matlabFunction(F, 'Vars', {[theta0, theta1, theta2], x, y, z});
% Esto crea una funcion equivalente F_handle = @(theta, x, y, z) ...

x_des = 30;
y_des = 30;
z_des = 60;

th0 = [0 0 0]; % guess inicial

lambda = 1e-2;
% DEFINICION DE FUNCION DE COSTO
cost_fun = @(theta) F_handle(theta, x_des, y_des, z_des);

% Definicion de restricciones
% lb y ub
lb = [0; 0; 0];
ub = [pi; pi; pi];

% Ahora se usa fmincon
theta_opt = fmincon(cost_fun,th0,[],[],[],[],lb,ub,[])

% Como px py pz los tengo como symbolic, uso subs para evaluarlos y double
% para que lo opere


double(subs(px, [theta0 theta1 theta2], theta_opt))
double(subs(py, [theta0 theta1 theta2], theta_opt))
double(subs(pz, [theta0 theta1 theta2], theta_opt))

%% Evaluacion preliminar de un punto
figure
P = fk_points(theta_opt);
plot3(P(1,:), P(2,:), P(3,:), 'o-', ...
          'LineWidth', 2, 'MarkerSize', 6) 
xlabel('X [cm]')
ylabel('Y [cm]')
zlabel('Z [cm]')
grid on
xlim([-20 80])
ylim([-30 30])
zlim([0 80])

%% Simulación de una trayectoria

% Definicion de la trayectoria
t = linspace(0,1,50);
x_traj = 30 + 10*t;
y_traj = 0*ones(size(t));
z_traj = 40 + 25*t; % aqui puede ir variando y viendo graficamente

% Este punto será el que se use como x0 cada vez que se ejecute el solver
theta_prev = [0 0 0];

% Buffers 
theta_hist = [];
fval_hist  = [];
flag_hist  = [];
time_hist  = [];

% obstaculo
cObs = [35;0;52];
Robs = 3; margin = 1; R = Robs + margin;
nonlcon_theta = @(theta) nonlcon_robot_sphere(theta, cObs, R);

lambda = 70;

for k = 1:length(t)

    x_des = x_traj(k);
    y_des = y_traj(k);
    z_des = z_traj(k);

    % Se define la funcion de costo para cada "instante k" 
    % ya que x_des, y_des, z_des no son variables de decision, pero van
    % variando cada instante k
    cost_fun = @(theta) F_handle(theta, x_des, y_des, z_des) ...
                  + lambda*sum((theta - theta_prev).^2);

    tic
    [theta_opt,fval,flag] = fmincon(cost_fun, theta_prev, ...
        [],[],[],[],lb,ub,nonlcon_theta);
    elapsed = toc;

    theta_prev = theta_opt;

    % Guardado
    theta_hist = [theta_hist; theta_opt];
    fval_hist  = [fval_hist;  fval];
    flag_hist  = [flag_hist;  flag];
    time_hist  = [time_hist;  elapsed];

end


%% Animación
video = VideoWriter('robot_trayectoria.mp4','MPEG-4');
video.FrameRate = 50;
open(video)

fig = figure('Color','w');
axis equal
grid on
view(3)
hold on

traj_real = [];   % buffer de trayectoria real

for k = 1:length(theta_hist)

    theta = theta_hist(k,:);
    P = fk_points(theta);
    ee = P(:,end);            % efector final actual
    traj_real = [traj_real ee];

    cla

    % brazo
    plot3(P(1,:), P(2,:), P(3,:), 'o-', ...
        'LineWidth',2,'MarkerSize',6)
    hold on

    % trayectoria deseada
    plot3(x_traj, y_traj, z_traj, 'r--')

    % trayectoria real con X
    plot3(traj_real(1,:), traj_real(2,:), traj_real(3,:), ...
        'bx','LineWidth',1,'MarkerSize',2)

    % obstaculo
    [sx,sy,sz] = sphere(30);
    surf(cObs(1)+Robs*sx, ...
         cObs(2)+Robs*sy, ...
         cObs(3)+Robs*sz, ...
         'FaceAlpha',0.2,'EdgeColor','none');

    xlabel('X [cm]')
    ylabel('Y [cm]')
    zlabel('Z [cm]')
    xlim([-20 80])
    ylim([-30 30])
    zlim([0 80])

    drawnow

    frame = getframe(gcf);
    writeVideo(video,frame);

end

close(video)
disp('Video guardado')

%%
% 1: El gradiente de la función de costo es prácticamente cero (El efector
% final llegó correctamente al punto deseado.)

% 2: El cambio en x fue menor que StepTolerance - Las variables (theta) ya
% casi no cambian (No se puede dar pasos mas pequeños)
% puede ser un estancamiento cerca del minimo

% 3: El cambio en el valor de la función objetivo fue menor que
% FunctionTolerance - El costo ya no disminuye de forma apreciable

% 5: La reducción prevista de la función objetivo fue menor que FunctionTolerance
% Casi ya no mejora si sigue iterando (analiza gradiente y hessiana)

