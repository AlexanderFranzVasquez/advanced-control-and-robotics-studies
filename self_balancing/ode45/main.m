clc; clear; close all;

% Physical parameters of the self-balancing robot

m = 1.3;          % kg Mass of the chassis (body of the robot)
M = 0.3;          % kg Mass of ONE wheel
R = 3.25/100;        % m Wheel radius

% Moment of inertia of the wheel about its rotation axis
% Wheel is modeled as a solid disk:
% Jw = (1/2) * M * R^2
Jw = 0.5 * M * R^2;   % kg·m^2

% Distance from the wheel axis (motor axis)
% to the center of mass of the chassis
l = 15/100;       % m

% Height of the chassis
% Used to approximate the chassis as a thin vertical bar
h = 28/100;       % m

% Moment of inertia of the chassis about the wheel axis
% The chassis is modeled as a thin rod (bar) of height h.
% First term: inertia of the bar about its center of mass
%   J_cm = (1/12) * m * h^2
% Second term: parallel axis theorem to shift inertia
% from the center of mass to the wheel axis
%   m * l^2
% Total inertia:
%   Jc = J_cm + m * l^2
Jc = (1/12) * m * h^2 + m * l^2;   % kg·m^2

% Damping
u0 = 0.1;
u1 = 0.1;

% Gravity 
g = 9.81; % m/s^2


% Create params for the model
params.m = m;          % kg Mass of the chassis (body of the robot)
params.M = M;          % kg Mass of ONE wheel
params.R = R;        % m Wheel radius
params.Jw = Jw;   % kg·m^2
params.l = l;       % m
params.h = h;       % m
params.Jc = Jc;   % kg·m^2
params.u0 = u0;
params.u1 = u1;
params.g = g; % m/s^2

params.Ts = 0.01;    % sampling time [s]
params.sigma = 0.1;  % std dev of measurement noise

%%

% Intermedial parameters

a = m + 2*M + 2* Jw/R^2;
b = m*l;
c = m*l^2 + Jc;
delta = a*c-b^2;

A = [0    1               0                0;
     0   -2*u0*c/delta   -b*m*g*l/delta    2*u1*b/delta;
     0    0               0                1;
     0    2*u0*b/delta    a*m*g*l/delta   -2*u1*a/delta];

B = [0;
     -b/delta;
     0;
     a/delta];

eig(A)

co = ctrb(A,B)

% Check controlability
rank(co) % 
% The complete system is not fully controllable due to translational invariance

Ared = A(2:4,2:4);
Bred = B(2:4);

rank(ctrb(Ared,Bred))   % 3
%%
% K - Optimal gain row vector
% S - Solution of the associated algebraic Riccati equation matrix
% P - Poles of the closed-loop system column vector

% Design of the controller for the reduced system

Q = [1 0 0;
     0 1 0;
     0 0 1];

R_ = 1;

[K,S,P] = lqr(Ared,Bred,Q,R_);

%%
% Simulation


dynOde45 = @(t,x) dynSelfBalancing(t,x,params,K);

tspan = 0:0.1:10; % 10 seconds

x0 = [0;
      0;
      -0.3;
      0];

[t,y] = ode45(dynOde45,tspan,x0);

x     = y(:,1);   % posición
xdot  = y(:,2);   % velocidad
theta = y(:,3);   % ángulo
thetadot = y(:,4);% velocidad angular


u = -K(1).*xdot - K(2).*theta - K(3).*thetadot;

%% PLOTS

figure(1)

subplot(2,2,1)
plot(t,x,'LineWidth',1.5)
grid on
xlabel('Time [s]')
ylabel('x [m]')
title('Position')

subplot(2,2,2)
plot(t,xdot,'LineWidth',1.5)
grid on
xlabel('Time [s]')
ylabel('dx/dt [m/s]')
title('Velocity')

subplot(2,2,3)
plot(t,theta,'LineWidth',1.5)
grid on
xlabel('Time [s]')
ylabel('\theta [rad]')
title('Angle')

subplot(2,2,4)
plot(t,thetadot,'LineWidth',1.5)
grid on
xlabel('Time [s]')
ylabel('d\theta/dt [rad/s]')
title('Angular velocity')

figure(2)

e_xdot = xdot;        % referencia 0
e_theta = theta;     % referencia 0
e_thetadot = thetadot;

subplot(3,1,1)
plot(t,e_xdot,'LineWidth',1.5)
grid on
ylabel('e_{x2}')

subplot(3,1,2)
plot(t,e_theta,'LineWidth',1.5)
grid on
ylabel('e_{x3}')

subplot(3,1,3)
plot(t,e_thetadot,'LineWidth',1.5)
grid on
ylabel('e_{x4}')
xlabel('Time [s]')

figure(3)
plot(t,u,'LineWidth',1.5)
grid on
xlabel('Time [s]')
ylabel('\tau [N·m]')
title('Control input')

%% ANIMATION

animateSelfBalancing(t, x, theta, R, h);
