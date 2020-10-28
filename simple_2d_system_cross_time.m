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
param.ncx = 2;
% start point
param.x0 = [0;0];     % always starts with 0
% goal point
param.xN = [3;param.x0(2)];   
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
% param.Qf = 500*eye(param.nx);
param.Qf = 0*eye(param.nx);

%% let's prepare a fancy plot to compare three methods
ftraj = figure(1); clf(ftraj);
ax_qp(1) = subplot(2, 3, 2);
ax_ln(1) = subplot(2, 3, 1);
ax_fg(1) = subplot(2, 3, 3);
ax_qp2(1) = subplot(2, 3, 5);
ax_ln2(1) = subplot(2, 3, 4);
ax_fg2(1) = subplot(2, 3, 6);
fctrl = figure(2); clf(fctrl);
ax_qp(2) = subplot(2, 3, 2);
ax_ln(2) = subplot(2, 3, 1);
ax_fg(2) = subplot(2, 3, 3);
ax_qp2(2) = subplot(2, 3, 5);
ax_ln2(2) = subplot(2, 3, 4);
ax_fg2(2) = subplot(2, 3, 6);
ftraj.Position(3:4) = [800, 400];

%% problem statement
% init constraint and state list
% array contains indices \in [1,N] that has constraint impose
constraint_pt = [2;-2];
param.Cxu = [];
param.Cx = [];
% param.Cx = [];
paramalt = param;
paramalt.Cx = [21, 41, 61, 81, 101];
% paramalt.Qf = 9999*eye(param.nx);
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
Galt_list = zeros(ncx, nx, N);
halt_list = zeros(ncx, N);
for i=1:N
    A_list(:,:,i) = param.A;
    B_list(:,:,i) = param.B;
    C_list(:,:,i) = zeros(ncxu, nx);
    D_list(:,:,i) = zeros(ncxu, nu);  % no state/control constraint in this case
    G_list(:,:,i) = zeros(ncx, nx);
    r_list(:,i) = zeros(ncxu, 1);
    h_list(:,i) = zeros(ncx,1);
end
ni = 1;
for n = paramalt.Cx
    Galt_list(:, :, n) = eye(2);
    halt_list(:, n) = [-0.6 * ni; 0];
    ni = ni + 1;
end
% G_list(:,:,param.Cx(1)) = eye(param.nx);
% G_list(:,:,param.Cx(2)) = eye(param.nx);
% actual constraint 
% h_list(:,param.Cx(1)) = -constraint_pt;
% h_list(:,param.Cx(2)) = -param.xN;

% cross time step constraints
S_list = {};Si = 1;
% add cross time constraint
for i=1:20:N-15
    S_list{Si} = struct();
    S_list{Si}.xn = [i, i+20];
    S_list{Si}.xmat(:, :, 1) = [1 0; 0 1];
    S_list{Si}.xmat(:, :, 2) = [-1 0; 0 -1];
    S_list{Si}.un = [];
    S_list{Si}.umat = zeros(2, nu, 0);
    S_list{Si}.rhs = [-0.6; -0];
    Si = Si + 1;
end
param.dimS = 2 * length(S_list);

%% closed loop control test
param_perturb = param;
param_perturb.x0(2) = param_perturb.x0(2)+1.8;
param_perturb.xN = [3;param_perturb.x0(2)]; 

% solve the LQR 
Soln = ecLQR_fg_cross(param, param.xN, A_list, B_list, C_list, D_list, G_list, r_list, h_list);% we know nx = 2
%% solve with quadprog
Soln = ecLQR_qp(param, param.xN, A_list, B_list, C_list, D_list, G_list, r_list, h_list, S_list);% we know nx = 2
xSol = zeros(1,N);
ySol = zeros(1,N);
uSol = zeros(nu,N);
for i=1:(N+1)
    xSol(i) = Soln(i).x(1);
    ySol(i) = Soln(i).x(2);
end
u = @(n, x) Soln(n).u;
for i=1:N
    uSol(:, i) = Soln(i).u;
end
[xSol2, ySol2, uSol2] = closedloop(param_perturb, u);
[cost_qp, vio_qp] = plotSol(ax_qp, '{\\bfBaseline Method 3}\nOptimal Trajectory\n\\itCost: %.2f - Constr: %.2e', param, xSol, ySol, uSol, A_list, B_list, C_list, D_list, G_list, r_list, h_list, S_list);
[cost_qp2, vio_qp2] = plotSol(ax_qp2, 'Perturbed Initial State\n\\itCost: %.2f - Constr: %.2e', param_perturb, xSol2, ySol2, uSol2, A_list, B_list, C_list, D_list, G_list, r_list, h_list, S_list);
qpsol = struct('xSol', xSol, 'ySol', ySol, 'uSol', uSol);
drawnow()

%% solve with Laine
Soln = ecLQR_laine(paramalt, paramalt.xN, A_list, B_list, C_list, D_list, Galt_list, r_list, halt_list);
xSol = zeros(1,N);
ySol = zeros(1,N);
for i=1:(N+1)
    xSol(i) = Soln(i).x(1);
    ySol(i) = Soln(i).x(2);
end
u = @(n, x) Soln(n).K * x(:, n) + Soln(n).k;
uSol = zeros(nu,N);
for i=1:N
    uSol(:,i) = Soln(i).K * Soln(i).x + Soln(i).k;
    uSol2(:,i) = u(i, [xSol;ySol]);
end
assert(all(abs(uSol - uSol2) < 1e-9), 'u function not correct');
[xSol2, ySol2, uSol2] = closedloop(param_perturb, u);
[cost_ln, vio_ln] = plotSol(ax_ln, '{\\bfBaseline Method 2}\nOptimal Trajectory\n\\itCost: %.2f - Constr: %.2e', param, xSol, ySol, uSol, A_list, B_list, C_list, D_list, G_list, r_list, h_list, S_list);
[cost_ln2, vio_ln2] = plotSol(ax_ln2, 'Trajectory with Perturbed Initial State\n\\itCost: %.2f - Constr: %.2e', param_perturb, xSol2, ySol2, uSol2, A_list, B_list, C_list, D_list, G_list, r_list, h_list, S_list);
drawnow();

%% solve the LQR with factor graph
Soln = ecLQR_fg_cross(param, param.xN, A_list, B_list, C_list, D_list, G_list, r_list, h_list, S_list);% we know nx = 2
xSol = zeros(1,N);
ySol = zeros(1,N);
for i=1:(N+1)
    xSol(i) = Soln(i).x(1);
    ySol(i) = Soln(i).x(2);
end
u = @(n, xs) -Soln(n).K * reshape(xs(:, Soln(n).nset), [], 1) + Soln(n).k;
uSol = zeros(nu,N);
for i=1:N
    if size(Soln(i).K,1) == 2
        Soln(i).K = Soln(i).K';
    end
    xs = reshape([Soln(Soln(i).nset).x], [], 1);
%     uSol(:,i) = -Soln(i).K * xs + Soln(i).k;
    uSol(:, i) = u(i, [xSol;ySol]);
end
[xSol2, ySol2, uSol2] = closedloop(param_perturb, u);
%%
sparsitypattern = zeros(N+1, N+1);
for t = 1:N+1
    sparsitypattern( t, Soln(t).nset ) = 1;
end
order = N+1:-1:1; sparsitypattern = sparsitypattern(order, order);
figure(5);clf;imshow(sparsitypattern~=0);
%%
[cost_fg, vio_fg] = plotSol(ax_fg, '{\\bfProposed Method}\nOptimal Trajectory\n\\itCost: %.2f - Constr: %.2e', param, xSol, ySol, uSol, A_list, B_list, C_list, D_list, G_list, r_list, h_list, S_list);
[cost_fg2, vio_fg2] = plotSol(ax_fg2, 'Trajectory with Perturbed Initial State\n\\itCost: %.2f - Constr: %.2e', param_perturb, xSol2, ySol2, uSol2, A_list, B_list, C_list, D_list, G_list, r_list, h_list, S_list);
fgsol = struct('xSol', xSol, 'ySol', ySol, 'uSol', uSol);

%% check
assert(abs(vio_fg - vio_qp) < 1e-12, 'constraint violations don''t match');
assert(abs(cost_fg - cost_qp) < 1e-9, 'costs don''t match');
assert(all(abs(fgsol.xSol - qpsol.xSol) < 1e-9), 'trajectories don''t match');
assert(all(abs(fgsol.ySol - qpsol.ySol) < 1e-9), 'trajectories don''t match');
assert(all(abs(fgsol.uSol - qpsol.uSol) < 1e-9), 'controls don''t match');
fprintf('solutions match\n');

%% plot
function [xSol, ySol, uSol] = closedloop(param, u)
    N = param.N;
    x = zeros(param.nx, N+1);
    uSol = zeros(param.nu, N);
    x(:, 1) = param.x0;
    for n = 1:N
        uSol(:, n) = u(n, x);
        x(:, n+1) = param.A*x(:, n) + param.B*uSol(:, n);
    end
    xSol = reshape(x(1, :), 1, []);
    ySol = reshape(x(2, :), 1, []);
end
function [finalcost, vio] = plotSol(ax, methodname, param, xSol, ySol, uSol, A_list, B_list, C_list, D_list, G_list, r_list, h_list, S_list)
    N = param.N;
    plot(ax(1), xSol,ySol,'r-','LineWidth',3);
    hold(ax(1), 'on');
    plot(ax(1), xSol(1), ySol(1), 'ro','MarkerSize',10,'LineWidth',2);
    % plot(ax(1), constraint_pt(1),constraint_pt(2),'go','MarkerSize',10,'LineWidth',3)
    for i = 1:5
        plot(ax(1), param.x0(1) + i*0.6, param.x0(2),'b*','MarkerSize',10,'LineWidth',3)
    end
    xLim = [-1,4];
    yLim = [-1,6];
    axis(ax(1), [xLim,yLim]); 
%     axis(ax(1), 'auto');
    finalcost = getCost(N,xSol,ySol,uSol,param.Q, param.R, param.Qf, param.xN);
    vio = getConViolate(N, param, [xSol;ySol], uSol, A_list, B_list, C_list, D_list, G_list, r_list, h_list, S_list);
    string = sprintf('final cost = %.2f constraint violation = %.2e', [finalcost, vio]);
    set(ax(1),'fontsize', 12)
    % title(string);
    string = sprintf(methodname, [finalcost, vio]);
    title(ax(1), string, 'FontSize', 12, 'FontWeight', 'normal');% without Cross-time-step Constraints');
    % title('Optimal Trajectory with Cross-time-step Constraints');
    grid(ax(1), 'on')
%     xlabel(ax(1), '$x_1$', 'interpreter', 'latex');ylabel(ax(1), '$x_2$', 'interpreter', 'latex');
%     xlabel(ax(1), 'x1'); ylabel(ax(1), 'x2');
    xlabel(ax(1), 'x1 (position)', 'interpreter', 'latex');ylabel(ax(1), 'x2 (velocity)', 'interpreter', 'latex');

    % subplot(1,2,2);
    % plot(1:N, xSol(1:N),1:N, ySol(1:N))
    % title('Proposed factor graph method state plot');
    % legend('x(1)','x(2)')
    plot(ax(2), 1:N, uSol)
%     plot(ax(2), 1:N+1, xSol);
    hold(ax(2), 'on');
%     plot(ax(2), 1:N+1, ySol);
    string2 = sprintf('\n control input plot');
    title(ax(2), string2);
    % legend('control')
    set(ax(2),'fontsize', 15)
    grid(ax(2), 'on')
    xlabel(ax(2), 'time step (n)', 'interpreter', 'latex'); ylabel(ax(2), 'u', 'interpreter', 'latex');
    
%     sgtitle({methodname, string});
end



