function Soln = ecLQR_qp(param, xN,  A_list, B_list, C_list, D_list, G_list, r_list, h_list, S_list)
%ECLQR_QR      solve LQR with equality constraint using quadratic programming method
%              in paper Efficient computation of feedback control for equality-constrained lqr
% It uses the general input output structure:
% Inputs:
%  -- param   a struct contains all necessary parameters
%  -- xN      The final target that state must reach at N
%  -- A_list B_list lists of system dynamics 
%  -- C_list, D_list, G_list, r_list, h_list lists of constraints 
% Outputs:
%  -- Soln 

%% constants
N = param.N;
nx = param.nx;
nu = param.nu;
ncxu = param.ncxu;    %lt
ncx = param.ncx;      %lT
dimS = param.dimS;

Cxu = param.Cxu;
Cx  = param.Cx;
Q = param.Q;
Qf = param.Qf;
R = param.R;

%% form big matrices
%   We need to form a QP problem of the form
%   argmin (-0.5*X'*H*X + f'*X)     s.t.  A*X = b
%   Assemble bigA/bigB from A, B, C, D, G, r, h
%   Assemble bigH/bigF from Q, R, Qf 
bigA = sparse(N*(nx+ncxu+ncx) + nx + dimS, N*(nx+nu)+nx); % extra equation for x0
bigB = sparse(N*(nx+ncxu+ncx) + nx + dimS, 1);

bigH = sparse(N*(nx+nu)+nx, N*(nx+nu)+nx); % last row for final state cost
bigF = sparse(N*(nx+nu)+nx, 1);

% convenience indexing matrices
t = (1:N)';
colMap = [ (t-1)*(nx+nu) + 1        ,... xstart
           (t-1)*(nx+nu) + nx       ,... xend
           (t-1)*(nx+nu) + nx + 1   ,... ustart
             t  *(nx+nu)            ]; % uend
colMap = [colMap; colMap(end, 1:2)+(nx+nu), -1,-1]; % x(T+1)
rowMapA =[ (t-1)*(nx+ncxu+ncx) + 1              ,... dynamics start
           (t-1)*(nx+ncxu+ncx) + nx             ,... dynamics end
           (t-1)*(nx+ncxu+ncx) + nx + 1         ,... C/D/r start
           (t-1)*(nx+ncxu+ncx) + nx + ncxu      ,... C/D/r end
           (t-1)*(nx+ncxu+ncx) + nx + ncxu + 1  ,... G/h start
             t  *(nx+ncxu+ncx)                  ]; % G/h end
rowMapA =[rowMapA; N*(nx+ncxu+ncx)+1, N*(nx+ncxu+ncx)+nx, -1,-1,-1,-1]; % x0
rowMapH =[ (t-1)*(nx+nu) + 1              ,... Q start
           (t-1)*(nx+nu) + nx             ,... Q end
           (t-1)*(nx+nu) + nx + 1         ,... R start
             t  *(nx+nu)                  ]; % R end
rowMapH =[rowMapH; N*(nx+nu)+1, N*(nx+nu)+nx, -1,-1]; % x(T+1)

% Ax+Bu = x_t+1 (dynamics)
for t = 1:N
    bigA(rowMapA(t, 1):rowMapA(t, 2), colMap(t, 1):colMap(t, 2)) = A_list(:, :, t);
    bigA(rowMapA(t, 1):rowMapA(t, 2), colMap(t, 3):colMap(t, 4)) = B_list(:, :, t);
    bigA(rowMapA(t, 1):rowMapA(t, 2), colMap(t+1, 1):colMap(t+1, 2)) = -eye(nx);
end

% prior
bigA(rowMapA(N+1, 1):rowMapA(N+1, 2), colMap(1, 1):colMap(1, 2)) = eye(nx);
bigB(rowMapA(N+1, 1):rowMapA(N+1, 2)) = param.x0;

% Cx+Du+r = 0
for t = 1:N
    if (ismember(t,param.Cxu))
    bigA(rowMapA(t, 3):rowMapA(t, 4), colMap(t, 1):colMap(t, 2)) = C_list(:, :, t);
    bigA(rowMapA(t, 3):rowMapA(t, 4), colMap(t, 3):colMap(t, 4)) = D_list(:, :, t);
    bigB(rowMapA(t, 3):rowMapA(t, 4)) = -r_list(:, t);
    end
end

% Gx + h = 0
for t = 1:N
    if (ismember(t,param.Cx))
    bigA(rowMapA(t, 5):rowMapA(t, 6), colMap(t, 1):colMap(t, 2)) = G_list(:, :, t);
    bigB(rowMapA(t, 5):rowMapA(t, 6)) = -h_list(:, t);
    end
end

% cross timestep constraints
row = max(rowMapA, [], 'all') + 1;
for S = S_list
    S = S{1};
    rowrange = row:row+size(S.rhs, 1)-1;
    for xi = 1:length(S.xn)
        t = S.xn(xi);
        bigA(rowrange, colMap(t, 1):colMap(t, 2)) = S.xmat(:, :, xi);
    end
    for ui = 1:length(S.un)
        t = S.un(ui);
        bigA(rowrange, colMap(t, 3):colMap(t, 4)) = S.umat(:, :, ui);
    end
    bigB(rowrange) = S.rhs;
    row = rowrange(end)+1;
end

% Q & R
for t = 1:N
    bigH(rowMapH(t, 1):rowMapH(t, 2), colMap(t, 1):colMap(t, 2)) = Q;
    bigH(rowMapH(t, 3):rowMapH(t, 4), colMap(t, 3):colMap(t, 4)) = R;
end
bigH(rowMapH(t+1, 1):rowMapH(t+1, 2), colMap(t+1, 1):colMap(t+1, 2)) = Qf;
bigF(rowMapH(t+1, 1):rowMapH(t+1, 2)) = -Qf*xN;

%% solve
sol = quadprog(bigH, bigF, [], [], bigA, bigB,...
               [], [], [], optimset('Display', 'off'));

Soln(N).t = 0;
Soln(N).u = zeros(nu,1);
Soln(N).x = zeros(nu,1);

for t=1:N
    Soln(t).x = sol(colMap(t, 1):colMap(t, 2));
    Soln(t).u = sol(colMap(t, 3):colMap(t, 4));
end
Soln(N+1).x = sol(colMap(N+1, 1):colMap(N+1, 2));

end