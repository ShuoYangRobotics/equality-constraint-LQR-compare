% init LQR problem, this is the simple example given by
% Laine 2019 section 3.B

clear; clc;
% state size
param.nx = 2;
% control size
param.nu = 1;
% state and control related constraint size
param.ncxu = 2;
% state only constraint size
param.ncx = param.nx;
% start point
param.x0 = [0;0];     % always starts with 0
% goal point
param.xN = [3;2];   
% total time
param.LQR_time = 1;
% dt for dicretizing
param.dt = 0.01;
dt = param.dt;
% total steps
param.N = param.LQR_time / param.dt;
N = param.N;
% system dynamics
param.A = [1 dt;
           0  1];
param.B = [0; 
           dt];

% running cost terms
param.Q = 1e-2*eye(param.nx);
param.R = 1e-3*eye(param.nu);
% final cost terms
param.Qf = 500*eye(param.nx);

%% let's prepare a fancy plot to compare three methods
figure(1); clf; hold on;
% init constraint and state list
% array contains indices \in [1,N] that has constraint impose
constraint_pt = [2;-2];
param.Cxu = [];
param.Cx = [param.N/2];
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
G_list(:,:,param.Cx(1)) = eye(param.nx);
% G_list(:,:,param.Cx(2)) = eye(param.nx);
% actual constraint 
h_list(:,param.Cx(1)) = -constraint_pt;
% h_list(:,param.Cx(2)) = -param.xN;

% for i=1:N   
%     if (ismember(i,param.Cxu))
%         C_list(:,:,i) = eye(param.nx);
%         D_list(:,:,i) = [1;1];
%         r_list(:,i) = [-1 1];
%     end
% end

% solve the LQR 
% unified input output struct 
%% 1. using sideris method
Soln_fg = ecLQR_sideris(param, param.xN, A_list, B_list, C_list, D_list, G_list, r_list, h_list);% we know nx = 2
xSol = zeros(1,N);
ySol = zeros(1,N);
for i=1:(N+1)
    xSol(i) = Soln_fg(i).x(1);
    ySol(i) = Soln_fg(i).x(2);
end
uSol = zeros(nu,N);
for i=1:N
    uSol(:,i) = Soln_fg(i).K * Soln_fg(i).x + Soln_fg(i).uff;
end
subplot(3,3,1); hold on;
plot(xSol,ySol,'r-','LineWidth',3);
plot(Soln_fg(1).x(1),Soln_fg(1).x(2),'ro','MarkerSize',10,'LineWidth',2)
% plot(constraint_pt(1),constraint_pt(2),'go','MarkerSize',10,'LineWidth',3)
plot(param.xN(1),param.xN(2),'b*','MarkerSize',10,'LineWidth',3)
xLim = [-5,5];
yLim = [-5,8];
axis([xLim,yLim]); axis equal;
finalcost = getCost(N,xSol,ySol,uSol,param.Q, param.R, param.Qf, param.xN);
vio = getConViolate(N, param, [xSol;ySol], uSol, C_list, D_list, G_list, r_list, h_list);
string = sprintf('Baseline Method 1 (Sideris) trajectory  \n final cost = %f  constraint violation = %f', [finalcost, vio]);
title(string);

subplot(3,3,4);
plot(1:N, xSol(1:N),1:N, ySol(1:N))
title('Baseline Method 1 state plot');
legend('x(1)','x(2)')
subplot(3,3,7);
plot(1:N, uSol)
title('Baseline Method 1 control plot');
legend('control')
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
subplot(3,3,3); hold on;
plot(xSol,ySol,'r-','LineWidth',3);
plot(Soln_fg(1).x(1),Soln_fg(1).x(2),'ro','MarkerSize',10,'LineWidth',2)
% plot(constraint_pt(1),constraint_pt(2),'go','MarkerSize',10,'LineWidth',3)
plot(param.xN(1),param.xN(2),'b*','MarkerSize',10,'LineWidth',3)
xLim = [-5,5];
yLim = [-5,8];
axis([xLim,yLim]); axis equal;
finalcost = getCost(N,xSol,ySol,uSol,param.Q, param.R, param.Qf, param.xN);
vio = getConViolate(N, param, [xSol;ySol], uSol, C_list, D_list, G_list, r_list, h_list);
string = sprintf('Proposed factor graph method trajectory  \n final cost = %f constraint violation = %f', [finalcost, vio]);
title(string);


subplot(3,3,6);
plot(1:N, xSol(1:N),1:N, ySol(1:N))
title('Proposed factor graph method state plot');
legend('x(1)','x(2)')
subplot(3,3,9);
plot(1:N, uSol)
title('Proposed factor graph method control plot');
legend('control')

%% 3. using Laine
Soln_l = ecLQR_laine(param, param.xN, A_list, B_list, C_list, D_list, G_list, r_list, h_list);
xSol = zeros(1,N);
ySol = zeros(1,N);
for i=1:(N+1)
    xSol(i) = Soln_l(i).x(1);
    ySol(i) = Soln_l(i).x(2);
end
uSol = zeros(nu,N);
for i=1:N
    uSol(:,i) = Soln_l(i).K * Soln_l(i).x + Soln_l(i).k;
end
subplot(3,3,2); hold on;
plot(xSol,ySol,'r-','LineWidth',3);
plot(Soln_l(1).x(1),Soln_l(1).x(2),'ro','MarkerSize',10,'LineWidth',2)
% plot(constraint_pt(1),constraint_pt(2),'go','MarkerSize',10,'LineWidth',3)
plot(param.xN(1),param.xN(2),'b*','MarkerSize',10,'LineWidth',3)
xLim = [-5,5];
yLim = [-5,8];
axis([xLim,yLim]); axis equal;
finalcost = getCost(N,xSol,ySol,uSol,param.Q, param.R, param.Qf, param.xN);
vio = getConViolate(N, param, [xSol;ySol], uSol, C_list, D_list, G_list, r_list, h_list);
string = sprintf('Baseline Method 2 (Laine) trajectory\n final cost =  %f  constraint violation = %f', [finalcost, vio]);
title(string);


subplot(3,3,5);
plot(1:N, xSol(1:N),1:N, ySol(1:N))
title('Baseline Method 2 state plot');
legend('x(1)','x(2)')
subplot(3,3,8);
plot(1:N, uSol)
title('Baseline Method 2 control plot');
legend('control')

%% plot result
% just plot solved trajectory


% simulate the system again using controller 










