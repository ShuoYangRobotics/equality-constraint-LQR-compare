clear; clc;
% state size
param.nx = 3;
% control size
param.nu = 3;
% state and control related constraint size
param.ncxu = 3;
% state only constraint size
param.ncx = param.nx;
% start point
param.x0 = [0;0;0];     % always starts with 0
% goal point
param.xN = [3;2;1];   
% total time
param.LQR_time = 1;
% dt for dicretizing
param.dt = 0.01;
dt = param.dt;
% total steps
param.N = param.LQR_time / param.dt;
N = param.N;
% system dynamics
% param.A = eye(3) + dt *[-0.4762    0.0576   -0.8775
%    -0.1532   -0.9880    0.0183
%    -0.8659    0.1432    0.4793];
% param.B = [-0.6294   -0.4978   -0.5967
%    -0.3749   -0.4781    0.7943
%    -0.6807    0.7236    0.1143]*dt;
param.A = eye(3)+eye(3)*dt;
param.B = eye(3)*dt;
% running cost terms
param.Q = 1e-2*eye(param.nx);
param.R = 1e-3*eye(param.nu);
% final cost terms
param.Qf = 500*eye(param.nx);

% this controls how much noise in the system simulation
param.simulation_noise = 0.0;

%% 
figure(1); clf; hold on;
constraint_pt = [1;2;3];
param.Cxu = [];
param.Cx = [param.N/2];
nx = param.nx;
nu = param.nu;
ncxu = param.ncxu;
ncx = param.ncx;
A_list = zeros(nx,nx,N);  % from 0 to N-1
B_list = zeros(nx,nu,N);  % from 0 to N-1
C_list = zeros(ncxu, nx, N); % from 0 to N-1
D_list = zeros(ncxu, nu, N); % from 0 to N-1
G_list = zeros(ncx, nx, N+1);
r_list = zeros(ncxu, N);
h_list = zeros(ncx, N+1);
for i=1:N
    A_list(:,:,i) = param.A;
    B_list(:,:,i) = param.B;
    C_list(:,:,i) = zeros(ncxu, nx);
    D_list(:,:,i) = zeros(ncxu, nu);  % no state/control constraint in this case
    G_list(:,:,i) = zeros(ncx, nx);
    r_list(:,i) = zeros(ncxu, 1);
    h_list(:,i) = zeros(ncx,1);
end

G_list(:,:,param.Cx(1)) = eye(param.nx);
h_list(:,param.Cx(1)) = -constraint_pt;

% solve the LQR 

font_size = 17;
%% 1. using sideris method
Soln_s = ecLQR_sideris(param, param.xN, A_list, B_list, C_list, D_list, G_list, r_list, h_list);% we know nx = 2
uSol = zeros(nu,N);
xSol = zeros(nx,N);
for i=1:(N+1)
    xSol(:,i) = Soln_s(i).x;
end
for i=1:N
    uSol(:,i) = Soln_s(i).K * Soln_s(i).x + Soln_s(i).uff;
end
subplot(1,3,1); hold on;
for i=1:nx
plot(1:N,xSol(i,1:N),'LineWidth',3);
end
% plot(1:N,uSol(i,1:N),'LineWidth',3);
finalcost_s = getCost_2(N,xSol,uSol,param.Q, param.R, param.Qf, param.xN);
vio_s = getConViolate(N, param, xSol, uSol, A_list, B_list, C_list, D_list, G_list, r_list, h_list, {});
string = sprintf('Baseline Method 1 \n final cost = %.2f \n constraint violation = %.2e', [finalcost_s, vio_s]);

% legend('x(1)','x(2)', 'x(3)','control')
legend('x(1)','x(2)', 'x(3)','Location','southeast')
set(gca,'fontsize', font_size)
title(string, 'FontSize', 18);
xlabel('Trajectory Steps','FontSize', 15)
ylabel('State Value','FontSize', 15)


%% 2. using Laine
Soln_l = ecLQR_laine(param, param.xN, A_list, B_list, C_list, D_list, G_list, r_list, h_list);
uSol = zeros(nu,N);
xSol = zeros(nx,N);
for i=1:(N+1)
    xSol(:,i) = Soln_l(i).x;
end
for i=1:N
    uSol(:,i) = Soln_l(i).K * Soln_l(i).x + Soln_l(i).k;
end
subplot(1,3,2); hold on;
for i=1:nx
plot(1:N,xSol(i,1:N),'LineWidth',3);
end
% plot(1:N,uSol(i,1:N),'LineWidth',3);
finalcost_l = getCost_2(N,xSol,uSol,param.Q, param.R, param.Qf, param.xN);
vio_l = getConViolate(N, param, xSol, uSol, A_list, B_list, C_list, D_list, G_list, r_list, h_list, {});
string = sprintf('Baseline Method 2 \n final cost = %.2f \n constraint violation = %.2e', [finalcost_l, vio_l]);
% legend('x(1)','x(2)', 'x(3)','control')
legend('x(1)','x(2)', 'x(3)','Location','southeast')
set(gca,'fontsize', font_size)
title(string, 'FontSize', 18);
xlabel('Trajectory Steps','FontSize', 15)
ylabel('State Value','FontSize', 15)

%% 3. using factor graph
Soln_fg = ecLQR_fg(param, param.xN, A_list, B_list, C_list, D_list, G_list, r_list, h_list);% we know nx = 2
uSol = zeros(nu,N);
xSol = zeros(nx,N);
for i=1:(N+1)
    xSol(:,i) = Soln_fg(i).x;
end
for i=1:N
    uSol(:,i) = Soln_fg(i).u;
%     uSol(:,i) = -Soln_fg(i).K * Soln_fg(i).x + Soln_fg(i).k;
end
subplot(1,3,3); hold on;
for i=1:nx
plot(1:N,xSol(i,1:N),'LineWidth',3);
end
% plot(1:N,uSol(i,1:N),'LineWidth',3);
finalcost_fg = getCost_2(N,xSol,uSol,param.Q, param.R, param.Qf, param.xN);
vio_fg = getConViolate(N, param, xSol, uSol, A_list, B_list, C_list, D_list, G_list, r_list, h_list, {});
string = sprintf('Proposed Method \n final cost = %.2f \n constraint violation = %.2e', [finalcost_fg, vio_fg]);

% legend('x(1)','x(2)', 'x(3)','control')
legend('x(1)','x(2)', 'x(3)','Location','southeast')
set(gca,'fontsize', font_size)
title(string, 'FontSize', 18);
xlabel('Trajectory Steps','FontSize', 15)
ylabel('State Value','FontSize', 15)
