% =========================================================================
% ALIP 2D - Simulación + Animación coherente con reset de origen
% =========================================================================

clear; clc; close all;

%% Parámetros del modelo
params = alip_params();
A = A_matrix(params);

m = params.m;
H = params.H;
T = params.T;

%% Estado inicial (marco local del pie actual)
x_c = 0.0;
L   = 0.0;

Nsteps = 80;

%% Buffers para historia
history.v_des = zeros(1,Nsteps);
history.vc_minus = zeros(1,Nsteps);
history.p = zeros(1,Nsteps);
history.pos_foot_world = zeros(1,Nsteps+1);

%% Figura de animación
fig = figure('Color','w');
axis equal; grid on; hold on;
xlabel('x (m)'); ylabel('z (m)');
xlim([-2 4]); ylim([0 1.6]);

% ================== VIDEO ==================
v = VideoWriter('ALIP_2D_animacion.mp4','MPEG-4');
v.FrameRate = 30;   % fps
open(v);

%% ===================== SIMULACIÓN =====================
for k = 1:Nsteps

    % -----------------------------------------------------
    % 1) Velocidad deseada (perfil por pasos)
    % -----------------------------------------------------
    if k <= 20
        v_des = 0.6;
    elseif k <= 40
        v_des = -0.3;
    elseif k <= 60
        v_des = 0.2;
    else
        v_des = 0.0;
    end

    history.v_des(k) = v_des;
    L_des = m * H * v_des;   % Aqui agregar (-) o en linea 64 o 115 -p

    % -----------------------------------------------------
    % 2) Dinámica ALIP durante el paso (ecuación 26)
    % -----------------------------------------------------
    X_minus = A * [x_c; L];
    x_minus = X_minus(1);
    L_minus = X_minus(2);

    history.vc_minus(k) = L_minus / (m*H);

    % -----------------------------------------------------
    % 3) Foot placement (ecuación 32)
    % -----------------------------------------------------
    p = foot_placement(L_minus, L_des, params)
    history.p(k) = p;

    % -----------------------------------------------------
    % 4) ANIMACIÓN DURANTE EL PASO (marco local + world shift)
    % -----------------------------------------------------
    n_frames = 10;
    x_world = history.pos_foot_world(k);

    for i = 1:n_frames
        alpha = i / n_frames;
        x_c_interp = (1-alpha)*x_c + alpha*x_minus;

        cla; hold on; grid on;
        xlim([-2 4]); ylim([0 1.6]);

        % Pie de apoyo (origen local)
        plot(x_world, 0, 'ks', 'MarkerSize',12, 'MarkerFaceColor','k');

        % CoM
        plot(x_world + x_c_interp, H, 'bo', ...
             'MarkerSize',10, 'MarkerFaceColor','b');

        % Péndulo
        plot([x_world, x_world + x_c_interp], [0 H], ...
             'b-', 'LineWidth',2);

        % Pie swing (posición deseada al impacto)
        plot(x_world + p, 0, 'ro', 'MarkerSize',9);

        title(sprintf('Paso %d',k));

        drawnow;                    % asegura actualización
        frame = getframe(fig);      % captura
        writeVideo(v, frame);       % guarda frame

        pause(0.03);
    end

    % -----------------------------------------------------
    % 5) RESET MAP (ecuación 25, marco local)
    % -----------------------------------------------------
    x_c = p;          % reset del origen (nuevo marco)
    L   = L_minus;

    % reconstrucción del mundo (solo para visualización)
    history.pos_foot_world(k+1) = history.pos_foot_world(k) + p;
end

close(v);
%% ===================== PLOTS =====================
figure(1);
hold on; 
grid on;
plot(history.vc_minus,'b','LineWidth',2);
plot(history.v_des,'r--','LineWidth',2);
xlabel('Paso k'); ylabel('m/s');
title('Velocidad real vs deseada');
legend('v_c','v_{des}');

figure(2); 
grid on;
plot(history.p,'LineWidth',2);
xlabel('Paso k'); ylabel('p (m)');
title('Foot placement p');

figure(3); 
grid on;
plot(history.pos_foot_world,'LineWidth',2);
xlabel('Paso k'); ylabel('x (m)');
title('Posición del pie en el mundo');

figure(4)
subplot(3,1,1)
hold on; 
grid on;
plot(history.vc_minus,'b','LineWidth',2);
plot(history.v_des,'r--','LineWidth',2);
xlabel('Paso k'); ylabel('m/s');
title('Velocidad real vs deseada');
legend('v_c','v_{des}');

subplot(3,1,2)
grid on;
plot(history.p,'LineWidth',2);
xlabel('Paso k'); ylabel('p (m)');
title('Foot placement p');

subplot(3,1,3)
grid on;
plot(history.pos_foot_world,'LineWidth',2);
xlabel('Paso k'); ylabel('x (m)');
title('Posición del pie en el mundo');