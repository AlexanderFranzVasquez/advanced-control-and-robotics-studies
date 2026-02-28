% Alexander F. Vasquez Montero
% IK with QP
% mainv5.m v5: Given a inital and final point the arm draw A CONTROLLED trajectory
% using Victor Paredes approach (with constraints in q)
% in this case, use as decision variables:
% qd
% and constraints:
% lb and ub constraints only for qd
% I embebed q into the constraint of qd
% As: q1 = q0 + dt*qd
% qd = (q1-q0)/dt
% in general:
% qd(k) = (q(k+1)-q(k))/dt    -> forward difference
% qd(k) = (q(k)-q(k-1))/dt    -> backward difference
% using forward difference, we try to embebed qmin and qmax
% given a qk, the max change is: qmax-qk , and the min change is qmin-qk
% we take a qmax or qmin as the next q, so, qd coul not take any valur
% lower or upper to that boundaries

% We introduce control to stop the arms when arrives at the final position

clc; clear; close all

% Definition according to the number of links and joints

n = 4; % Number of DoF
theta = sym('theta',[n 1]);

% Physic parameter of the robot arm

d1 = 10; % cm
a2 = 50; % cm
a3 = 30; % cm
a4 = 10; % cm

% Params: alpha a d theta

dh0t1 = [pi/2,  0, d1, theta(1)];
dh1t2 = [   0, a2,  0, theta(2)];
dh2t3 = [   0, a3,  0, theta(3)];
dh3t4 = [pi/2, a4,  0, theta(4)];

% Transforms

T01 = dh(dh0t1);
T12 = dh(dh1t2);
T23 = dh(dh2t3);
T34 = dh(dh3t4);

% FK

T04 = T01*T12*T23*T34;
T04 = simplify(T04);

% Position of the end effector

pe = [T04(1,4);
     T04(2,4);
     T04(3,4)];
fk = matlabFunction(pe,'Vars',{theta},'File','auto_Fk');

% Compute the Jacobian (Linear velocity) of the jacobian matrix

Jv = jacobian(pe,theta); % Jv(qk)

% As the jacobian is nonlinear, we need to evaluate Jv(qk) in an specific
% qk in time tk 

Jv = matlabFunction(Jv,'Vars',{theta},'File','auto_Jv');

% As the expression for the IK is:
% Jv*qd = ve
% The cost function is ||Jv*qd-ve||^2 + lambda*||qd||^2
% subject to: Bound contrains for q and qd

% --- Trajectory ---
ee0 = [90;
       0;
       10]; % theta = [0 0 0 0]^T

eef = [30;
       20;
       70];

% Parameter of the trajectory (in this case only with a constant velocity vd)

% Time desired

t = 10; % seconds

% Velocity desired

vd = sym('vd',[3 1],'real');   % velocidad cartesiana
% vd = (eef-ee0)./t; % cm/s (ideal velocity)

% Parameters for the cost function

lambda = 0.1;

% Compute H f A b

qk = sym('qk',[n 1],'real'); % we create this only to compute the H f for quadprog
Jvqk = Jv(qk); % pre operation

% If we compute the gradient and hessian of the cost function we can
% obtatin a expression for H and f

H = 2*(Jvqk.'*Jvqk+lambda*eye(n)); % explicit expressions for H f 
f = -2*Jvqk.'*vd;

H = simplify(H);
f = simplify(f);

%% For version 4.2
% Approach: embebed information

H = matlabFunction(H,'Vars',{qk},'File','auto_H');
f = matlabFunction(f,'Vars',{qk, vd},'File','auto_f');

% Constraints
qd_max = 1; % if 1 rad/s, it has more relevance than qmin

A = []; b = []; Aeq = []; beq = []; lb = []; ub = [];

% Lb and ub constraints (New)
qmin = 0*ones(n,1);
qmax = pi*ones(n,1);

dt = 0.1; % sampling time
% lb = @(qk) (qmin-qk)./dt;
% ub = @(qk) (qmax-qk)./dt;
                

lb = @(qk) max(-qd_max*ones(n,1), (qmin - qk)/dt);
ub = @(qk) min( qd_max*ones(n,1), (qmax - qk)/dt);

% ---- Pre-compute finish

%% Algorithm 

% Initialization is not arbitrary, it is subject to the physics

xd = eef;

k = 1;

qk = [0;
      0;
      0;
      0];

dt = 0.1; % sampling time
t = 0:dt:10; % Simulating for 10 s 

% Buffers
flags = [];
qk_hist = qk;
qd_hist = [];
vk_hist = [];
e_hist = [];

for i = t
    tic

    % Forward kinematics
    x = fk(qk); % Current end effector position

    % Compute error
    e = xd-x;
    
    % Desired velocity
    vk = k*e;

    [xopt, fval, exitflag] = quadprog(H(qk),f(qk,vk),A,b,Aeq,beq,lb(qk),ub(qk));

    qdopt = xopt;
    % Update the joint angles based on the optimized joint velocities
    qk = qk + qdopt * dt; % Forward euler integration 

    % Update buffers
    flags = [flags exitflag];
    qk_hist = [qk_hist qk];
    qd_hist = [qd_hist qdopt];
    vk_hist = [vk_hist vk];
    e_hist = [e_hist e];

    toc
end

%% Animation

video = VideoWriter('robot_qp.mp4','MPEG-4');
video.FrameRate = 50;

open(video)

params = [d1 a2 a3 a4];

fig = figure('Color','w');
axis equal
grid on
xlabel('X')
ylabel('Y')
zlabel('Z')
view(3)

xlim([-100 100])
ylim([-100 100])
zlim([0 150])

hold on

% Initialization of the robot
P = fk_points(qk_hist(:,1), params);  % qk0

h = plot3(P(1,:), P(2,:), P(3,:), ...
          '-o', 'LineWidth', 2, 'MarkerSize', 6);

% Init trajectory
ee_traj = zeros(3, size(qk_hist,2));
ee_traj(:,1) = P(:,end);

traj = plot3(ee_traj(1,1), ee_traj(2,1), ee_traj(3,1), ...
             'r', 'LineWidth', 1.5);

pause(1)

for k = 2:size(qk_hist,2)

    % FK
    P = fk_points(qk_hist(:,k), params);

    % Save position the FE
    ee_traj(:,k) = P(:,end);

    % Update body robot
    set(h, 'XData', P(1,:), ...
           'YData', P(2,:), ...
           'ZData', P(3,:));

    % Actualizar trayectoria del efector final
    set(traj, 'XData', ee_traj(1,1:k), ...
              'YData', ee_traj(2,1:k), ...
              'ZData', ee_traj(3,1:k));

    drawnow
    pause(0.05)

    frame = getframe(gcf);
    writeVideo(video,frame);

end


close(video)
disp('Video guardado')

figure(2)

hold on; grid on;

plot(t, qk_hist(1,1:end-1), '-o', 'LineWidth', 1.5);
plot(t, qk_hist(2,1:end-1), '-o', 'LineWidth', 1.5);
plot(t, qk_hist(3,1:end-1), '-o', 'LineWidth', 1.5);
plot(t, qk_hist(4,1:end-1), '-o', 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Joint angle [rad]');
title('Joint trajectories');

legend('q_1','q_2','q_3','q_4');

figure(3);
plot(t, qd_hist, '-o', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Joint velocity [rad/s]');
title('Joint velocity trajectories');
legend('qd_1','qd_2','qd_3','qd_4');

figure(4);
plot(t, vk_hist, '-o', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Velocity [rad/s]');
title('End Effector Velocity');
legend('x','y','z');

figure(5);
plot(t, e_hist, '-o', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Error [rad]');
title('Error');
legend('x','y','z');

figure(6)

subplot(4,1,1)
hold on; grid on;

plot(t, qk_hist(1,1:end-1), '-o', 'LineWidth', 1.5);
plot(t, qk_hist(2,1:end-1), '-o', 'LineWidth', 1.5);
plot(t, qk_hist(3,1:end-1), '-o', 'LineWidth', 1.5);
plot(t, qk_hist(4,1:end-1), '-o', 'LineWidth', 1.5);

xlabel('Time [s]');
ylabel('Joint angle [rad]');
title('Joint trajectories');

legend('q_1','q_2','q_3','q_4');

subplot(4,1,2)
plot(t, qd_hist, '-o', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Joint velocity [rad/s]');
title('Joint velocity trajectories');
legend('qd_1','qd_2','qd_3','qd_4');

subplot(4,1,3)
plot(t, vk_hist, '-o', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Velocity [rad/s]');
title('End Effector Velocity');
legend('x','y','z');

subplot(4,1,4)
plot(t, e_hist, '-o', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Error [rad]');
title('Error');
legend('x','y','z');
%% Future work:

% Past issues (v3)

% As we can see, the arm goes off the operation region (considering the
% space plot as operation region) it is necessary to constraint q values
% as the robot is redundant, it is possible

% Solution:

% the arm operate in a "safe" region but we constraint the q values, not
% position directly, this approach expand the variable decision vector for
% the optimization

% Past issues (v4,v4.2): We have error in the final position
% Solution: Control

% Implement CONTROL BARRIER FUNCTIONS