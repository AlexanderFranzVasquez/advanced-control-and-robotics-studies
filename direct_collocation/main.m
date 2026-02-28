clc; clear; close all;

% Problema de trajectory optimization para un péndulo en un carrito
% Se resuelve utilizando fmincon

% Se crea una funcion de costo en el archivo fcost

% Creo las retricciones no lineales en el archivo nonlcon

umax = 20;
dmax = 2;
T = 2; % s
N = 100; % N muestras, cambiar en fcost y nonlcon
deltaT = T/(N-1);

d = 1;    % m
m1 = 1;   % kg
m2 = 0.3; % kg
g = 9.81; % m/s^2
l = 0.5; % m


lb_u = -umax*ones(N,1);
lb_z1 = -dmax*ones(N,1);
lb_z2 = -inf*ones(N,1);
lb_z3 = -inf*ones(N,1);
lb_z4 = -inf*ones(N,1);

ub_u = umax*ones(N,1);
ub_z1 = dmax*ones(N,1);
ub_z2 = inf*ones(N,1);
ub_z3 = inf*ones(N,1);
ub_z4 = inf*ones(N,1);

lb = [lb_u;
      lb_z1;
      lb_z2;
      lb_z3;
      lb_z4];

ub = [ub_u;
      ub_z1;
      ub_z2;
      ub_z3;
      ub_z4];


% x0 es una trayectoria tanto u z1 z2 z3 z4 
uinit = zeros(N,1);
% z1 debe de llegar a d y empieza en cero
z1_init = linspace(0,d,N)';

% z2 debe de llegar a pi y empieza en cero
z2_init = linspace(0,pi,N)';

z3_init = zeros(N,1);

z4_init = zeros(N,1);

x0 = [uinit;
      z1_init;
      z2_init;
      z3_init;
      z4_init];

opts = optimoptions('fmincon',...
    'Algorithm','interior-point',...
    'Display','iter',...
    'MaxFunctionEvaluations',1e6,...
    'MaxIterations',1e4,...
    'ConstraintTolerance',1e-6,...
    'OptimalityTolerance',1e-6,...
    'StepTolerance',1e-10);

[x,fval,exitflag] = fmincon(@fcost,x0,[],[],[],[],lb,ub,@nonlcon,opts);

u  = x((0*N+1):1*N);
z1 = x((1*N+1):2*N);
z2 = x((2*N+1):3*N);
z3 = x((3*N+1):4*N);
z4 = x((4*N+1):5*N);

t = 0:deltaT:T;

figure(1)
plot(t,u)

figure(2)
plot(t,z1)

figure(3)
plot(t,z2)