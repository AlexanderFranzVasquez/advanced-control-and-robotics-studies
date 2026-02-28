%% SIMULACIÓN

% Crea los t nodes que le corresponderán a cada u,
% crea de 0 a T, una cantidad de N nodes
t_nodes = linspace(0,T,N);

% Interpolación del control (para tiempo continuo, tu planeamiento estaba en discreto)
u_fun = @(t) interp1(t_nodes, u, t, 'linear', 'extrap');

% Dinámica del cart-pole (ODE), ya tenias un cartpole_ode pero no en el
% formato que pide ode45, con esto, puedes mandarle mas parametros y luego
% usar la funcion adecuada para ode45
dyn = @(t,x) cartpole_ode(t,x,u_fun,m1,m2,g,l);

% Condición inicial (basado en el planeamiento NLP)
x0 = [z1(1); z2(1); z3(1); z4(1)];

% Simulación continua
opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
[t,x] = ode45(dyn, [0 T], x0, opts);

%% ANIMACION Y captura de video 
fig = figure('Color','w');
clf;
axis equal
axis([-1.5 1.5 -1.2 1.2])
grid on; hold on;

% Geomtria del carro a dibujar w: 
% ancho h: altura
cart_w = 0.3;
cart_h = 0.15;

% Inicializacion en 0,0 del carrito
% esquina inferior (0,0) y de ahi le pasas ancho y altura
% Position = [x  y  width  height]
cart = rectangle('Position',[0 0 cart_w cart_h],...
                 'FaceColor',[0 0.4 0.8]);

% Inicializacion de la polea y la masa
pole = plot([0 0],[0 0],'r','LineWidth',3);
mass = plot(0,0,'ro','MarkerSize',10,'MarkerFaceColor','r');

% Para dibujar trayectoria
frame_step = 5;
px_traj = [];
py_traj = [];

traj = plot(nan, nan, 'b-', 'LineWidth', 1);

%%%%%%%%%%%%%%% PARA VIDEO %%%%%%%%%%%%%%%%%%%%%%%%%
video_name = 'cart_pole_animation.mp4';

video = VideoWriter(video_name, 'MPEG-4');
video.FrameRate = 30;     % frames por segundo
video.Quality   = 100;    % calidad máxima

open(video);
%%%%%%%%%%%%%%% ---------- %%%%%%%%%%%%%%%%%%%%%%%%%

for k = 1:frame_step:length(t)

    x_cart = x(k,1);
    th     = x(k,2);

    % posicion de la masa de la polea, tu control solo te da th, tu sacas
    % px py geometricamente
    px = x_cart + l*sin(th);
    py = cart_h/2 - l*cos(th);
    
    % Para dibujar trayectoria
    px_traj(end+1) = px;
    py_traj(end+1) = py;
    
    set(traj,'XData',px_traj,'YData',py_traj);
    % traj.XData = px_traj;
    % traj.YData = py_traj;

    plot(px_traj,py_traj)
    % carrito (lo que vas actualizando es x_cart)
    set(cart,'Position',[x_cart-cart_w/2, 0, cart_w, cart_h]);

    % péndulo
    set(pole,'XData',[x_cart px],'YData',[cart_h/2 py]);
    set(mass,'XData',px,'YData',py);

    drawnow;

    %%%%%%%%%%%%%%% PARA VIDEO %%%%%%%%%%%%%%%%%%%%%%%%%
    frame = getframe(gcf);
    writeVideo(video, frame);
    %%%%%%%%%%%%%%% ---------- %%%%%%%%%%%%%%%%%%%%%%%%%

end
close(video);
%% PLOTEO DE GRAFICAS
% Para reconstruir la señal de control en funcion del tiempo de simulacion
% que me da ode45 (ode45 no devuelve la señal de control usada, solo
% estados)
% Cuando se usa ode45 lo normal es reconstruir luego la ley de control
% Si el control tiene dinamica propia, puede entrar como un estado udot = u
% + ....
u_cont = arrayfun(u_fun, t);
figure; clf;

% z1: posición del carrito 
subplot(3,1,1)
plot(t, x(:,1), 'b-', 'LineWidth',1.5); hold on   % grafica continua de ode45
plot(t_nodes, z1, 'ko', 'MarkerSize',5, 'LineWidth',1.2) % grafica discreta del planeamiento  (NLP)
grid on
ylabel('z_1 = x')
legend('ODE continuo','NLP nodos','Location','best')

% z2: ángulo del péndulo 
subplot(3,1,2)
plot(t, x(:,2), 'b-', 'LineWidth',1.5); hold on  % grafica continua de ode45
plot(t_nodes, z2, 'ko', 'MarkerSize',5, 'LineWidth',1.2) % grafica discreta del planeamiento  (NLP)
grid on
ylabel('z_2 = \theta')

% u: control 
subplot(3,1,3)
plot(t, u_cont, 'r-', 'LineWidth',1.5); hold on % grafica continua de ode45
plot(t_nodes, u, 'ks', 'MarkerSize',5, 'LineWidth',1.2)  % grafica discreta del planeamiento  (NLP)
grid on
ylabel('u')
xlabel('time [s]')
legend('u(t) continuo','u NLP nodos','Location','best')