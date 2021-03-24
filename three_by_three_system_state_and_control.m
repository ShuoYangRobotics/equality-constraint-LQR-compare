% init LQR problem, this is the simple example given by
% Laine 2019 section 3.B

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

% 100 
% Elapsed time is 0.007005 seconds.
% Elapsed time is 0.008632 seconds.
% 200
% Elapsed time is 0.009016 seconds.
% Elapsed time is 0.015318 seconds.
% 300
% Elapsed time is 0.014324 seconds.
% Elapsed time is 0.024785 seconds.
% 400
% Elapsed time is 0.017100 seconds.
% Elapsed time is 0.028869 seconds.
% 500
% Elapsed time is 0.021718 seconds.
% Elapsed time is 0.036138 seconds.
% 600
% Elapsed time is 0.026662 seconds.
% Elapsed time is 0.042953 seconds.

% dt for dicretizing
param.dt = 0.01;
dt = param.dt;
% total steps
param.N = param.LQR_time / param.dt;
N = param.N;
% system dynamics
param.A = eye(3) + dt *[-0.4762    0.0576   -0.8775
   -0.1532   -0.9880    0.0183
   -0.8659    0.1432    0.4793];
param.B = [-0.6294   -0.4978   -0.5967
   -0.3749   -0.4781    0.7943
   -0.6807    0.7236    0.1143]*dt;
% param.A = eye(3)+eye(3)*dt;
% param.B = eye(3)*dt;
% running cost terms
param.Q = 1e-2*eye(param.nx);
param.R = 1e-3*eye(param.nu);
% final cost terms
param.Qf = 500*eye(param.nx);

% this controls how much noise in the system simulation
param.simulation_noise = 0;
dynamicsnoise = randn(N, param.nx)*param.simulation_noise;

%% 
figure(1); clf; hold on;
param.Cxu = [param.N/2];
param.Cx = [];
% param.Cx = [];
nx = param.nx;
nu = param.nu;
ncxu = param.ncxu;
ncx = param.ncx;
A_list = zeros(nx,nx,N);  % from 0 to N-1
B_list = zeros(nx,nu,N);  % from 0 to N-1
C_list = zeros(ncxu, nx, N); % from 0 to N-1
D_list = zeros(ncxu, nu, N); % from 0 to N-1
G_list = zeros(ncx, nx, N);
r_list = zeros(ncxu, N);
h_list = zeros(ncx, N);
for i=1:N
    A_list(:,:,i) = param.A;
    B_list(:,:,i) = param.B;
    C_list(:,:,i) = zeros(ncxu, nx);
    D_list(:,:,i) = zeros(ncxu, nu);  % no state/control constraint in this case
    G_list(:,:,i) = zeros(ncx, nx);
    r_list(:,i) = zeros(ncxu, 1);
    h_list(:,i) = zeros(ncx,1);
end


for i=1:N   
    if (ismember(i,param.Cxu))
        C_list(:,:,i) = eye(param.nx);
        D_list(:,:,i) = [1 0 0
                         0 1 0
                         0 0 1];
        r_list(:,i) = [10 20 30];
    end
end

% solve the LQR 

font_size = 14;
%% 1. using Laine

% Soln_l 
[Solnl_x, Solnl_K, Solnl_k] = ecLQR_laine(param, param.xN, A_list, B_list, C_list, D_list, G_list, r_list, h_list);

xSol = zeros(1,N);
ySol = zeros(1,N);
for i=1:(N+1)
    xSol(i) = Solnl_x(1,i);
    ySol(i) = Solnl_x(2,i);
end
uSol = zeros(nu,N);
for i=1:N
    uSol(:,i) = Solnl_K(:,:,i) * Solnl_x(:,i) + Solnl_k(:,i);
end
subplot(1,2,1); hold on;
% plot(xSol,ySol,'r-','LineWidth',3);
% plot(Soln_l(1).x(1),Soln_l(1).x(2),'ro','MarkerSize',10,'LineWidth',2)
% % plot(constraint_pt(1),constraint_pt(2),'go','MarkerSize',10,'LineWidth',3)
% plot(param.xN(1),param.xN(2),'b*','MarkerSize',10,'LineWidth',3)
% xLim = [-5,5];
% yLim = [-5,8];
% axis([xLim,yLim]); axis equal;


% plot controller
for i =1:nu
    for j = 1:nx
        plot(1:N, squeeze(Solnl_K(i,j,:)),'LineWidth',3); hold on;
        plot(1:N, Solnl_k(i,:),'LineWidth',3); hold on;
    end
end
% plot(1:N, K_list(1,:),'r',1:N, K_list(2,:),'g',1:N, k_list,'b')
string = sprintf('Baseline Method 2 \n Elements in optimal controller K and k');
title(string);
% legend('K(1)','K(2)','k')
set(gca,'fontsize', font_size)
xlabel('Trajectory Steps','FontSize', 16)
ylabel('Element Value','FontSize', 16)
controller_laine.K = Solnl_K;
controller_laine.k = Solnl_k;


% subplot(3,2,3);
% plot(1:N, xSol(1:N),'r',1:N, ySol(1:N),'g',1:N, uSol,'b')
% finalcost = getCost(N,xSol,ySol,uSol,param.Q, param.R, param.Qf, param.xN);
% vio = getConViolate(N, param, [xSol;ySol], uSol, C_list, D_list, G_list, r_list, h_list);
% string = sprintf('Baseline Method 2 solved optimal trajectory plot, u = Kx+k\n final cost =  %f  constraint violation = %f', [finalcost, vio]);
% title(string);
% legend('x(1)','x(2)', 'control')
% set(gca,'fontsize', 12)

% simulate the system
% rng(10);
% x = param.x0;
% sim_x_list = zeros(nx,N+1);
% sim_u_list = zeros(nu,N);
% for i=1:N
%     sim_x_list(:,i) = x;
%     sim_u_list(:,i) = Soln_l(i).K * x + Soln_l(i).k;
%     x = param.A*x + param.B*sim_u_list(:,i) + dynamicsnoise(i, :)';
% end
% sim_x_list(:,N+1) = x;
% 
% finalcost_l = getCost_2(N,sim_x_list,sim_u_list,param.Q, param.R, param.Qf, param.xN);
% vio_l = getConViolate(N, param, sim_x_list, sim_u_list, C_list, D_list, G_list, r_list, h_list);
% subplot(2,2,3);
% plot(1:N, sim_x_list(1,1:end-1),'r',1:N, sim_x_list(2,1:end-1),'g',1:N, sim_x_list(3,1:end-1),'b')
% string = sprintf('Simulated trajectory and control plot, \n final cost =  %.2f  violation = %.2e', [finalcost_l, vio_l]);
% title(string);
% legend('x(1)','x(2)', 'x(3)', 'Location', 'northwest')
% set(gca,'fontsize', font_size)


%% 2. using factor graph

Soln_fg = ecLQR_fg(param, param.xN, A_list, B_list, C_list, D_list, G_list, r_list, h_list);% we know nx = 2

xSol = zeros(1,N);
ySol = zeros(1,N);
for i=1:(N+1)
    xSol(i) = Soln_fg(i).x(1);
    ySol(i) = Soln_fg(i).x(2);
end
uSol = zeros(nu,N);
for i=1:N
    uSol(:,i) = -Soln_fg(i).K * Soln_fg(i).x + Soln_fg(i).k;
end
subplot(1,2,2); hold on;
% plot(xSol,ySol,'r-','LineWidth',3);
% plot(Soln_fg(1).x(1),Soln_fg(1).x(2),'ro','MarkerSize',10,'LineWidth',2)
% % plot(constraint_pt(1),constraint_pt(2),'go','MarkerSize',10,'LineWidth',3)
% plot(param.xN(1),param.xN(2),'b*','MarkerSize',10,'LineWidth',3)
% xLim = [-5,5];
% yLim = [-5,8];
% axis([xLim,yLim]); axis equal;
% finalcost = getCost(N,xSol,ySol,uSol,param.Q, param.R, param.Qf, param.xN);
% vio = getConViolate(N, param, [xSol;ySol], uSol, C_list, D_list, G_list, r_list, h_list);
% string = sprintf('Proposed factor graph method trajectory  \n final cost = %f constraint violation = %f', [finalcost, vio]);
% title(string);

% plot controller
K_list = zeros(nu,nx,N);
k_list = zeros(nu,N);
for i=1:N
    K_list(:,:,i) = Soln_fg(i).K;
    k_list(:,i) = Soln_fg(i).k;
end
controller_fg.K = K_list;
controller_fg.k = k_list;

for i =1:nu
    for j = 1:nx
        plot(1:N, squeeze(K_list(i,j,:)),'LineWidth',3); hold on;
        plot(1:N, k_list(i,:),'LineWidth',3); hold on;
    end
end
% plot(1:N, K_list(1,:),'r',1:N, K_list(2,:),'g',1:N, k_list,'b')
% 
string = sprintf('Proposed method \n Elements in optimal controller K and k');
title(string);
% legend('K(1)','K(2)','k')
set(gca,'fontsize', font_size)
xlabel('Trajectory Steps','FontSize', 16)
ylabel('Element Value','FontSize', 16)

% subplot(3,2,4);
% plot(1:N, xSol(1:N),'r',1:N, ySol(1:N),'g',1:N, uSol,'b')
% finalcost = getCost(N,xSol,ySol,uSol,param.Q, param.R, param.Qf, param.xN);
% vio = getConViolate(N, param, [xSol;ySol], uSol, C_list, D_list, G_list, r_list, h_list);
% string = sprintf('Proposed method solved optimal trajectory plot, u = Kx+k\n final cost =  %f  constraint violation = %f', [finalcost, vio]);
% title(string);
% legend('x(1)','x(2)', 'control')
% set(gca,'fontsize', 12)

% simulate the system
% rng(10);
% x = param.x0;
% sim_x_list = zeros(nx,N+1);
% sim_u_list = zeros(nu,N);
% for i=1:N
%     sim_x_list(:,i) = x;
%     sim_u_list(:,i) = -Soln_fg(i).K * x + Soln_fg(i).k;
%     x = param.A*x + param.B*(sim_u_list(:,i)) + dynamicsnoise(i, :)';
% end
% sim_x_list(:,N+1) = x;
% 
% finalcost_fg = getCost_2(N,sim_x_list,sim_u_list,param.Q, param.R, param.Qf, param.xN);
% vio_fg = getConViolate(N, param, sim_x_list, sim_u_list, C_list, D_list, G_list, r_list, h_list);
% subplot(2,2,4);
% plot(1:N, sim_x_list(1,1:end-1),'r',1:N, sim_x_list(2,1:end-1),'g',1:N, sim_x_list(3,1:end-1),'b')
% string = sprintf('Simulated trajectory and control plot, \n final cost =  %.2f  violation = %.2e', [finalcost_fg, vio_fg]);
% title(string);
% legend('x(1)','x(2)', 'x(3)', 'Location', 'northwest')
% set(gca,'fontsize', font_size)

% dlmwrite('test.csv',[finalcost_l, vio_l,finalcost_fg, vio_fg],'delimiter',',','-append');
% simulate the system again using controller 

%% check if two controllers are identical
assert(all(abs(controller_laine.K + controller_fg.K) < 1e-6, 'all'),...
      'controller gains K not identical');
assert(all(abs(controller_laine.k - controller_fg.k) < 1e-6, 'all'),...
      'controller affine offsets k not identical');
warn(all(abs(controller_laine.K + controller_fg.K) < 1e-9, 'all'),...
      'controller gains K a little bit off');
warn(all(abs(controller_laine.k - controller_fg.k) < 1e-9, 'all'),...
      'controller affine offsets k a little bit off');
fprintf('The controllers for Laine and FG are identical\n');

function [] = warn(cond, msg)
    if (~cond)
        warning([msg,'\n']);
    end
end