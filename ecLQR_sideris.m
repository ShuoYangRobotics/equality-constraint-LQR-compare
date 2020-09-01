function Soln = ecLQR_sideris(param, xN,  A_list, B_list, C_list, D_list, G_list, r_list, h_list);
%ECLQR_SIDERIS solve LQR with equality constraint using sideris method
%              in paper A riccati approach to equality constrained linear quadratic optimal control
% It uses the general input output structure:
% Inputs:
%  -- param   a struct contains all necessary parameters
%  -- xN      The final target that state must reach at N
%  -- A_list B_list lists of system dynamics 
%  -- C_list, D_list, G_list, r_list, h_list lists of constraints 
% Outputs:
%  -- Soln 

%1. necessary variables 
N = param.N;
nx = param.nx;
nu = param.nu;
ncxu = param.ncxu;
ncx = param.ncx;

Cxu = param.Cxu;
Cx  = param.Cx;
Q = param.Q;
Qf = param.Qf;
R = param.R;

% 2. cacluate a number of hat matrices K = 0 x_0 = 0 u_0 = 0
Dhat_list = zeros(ncxu, ncxu, N);
E_list = zeros(ncxu,nx,N);
rhat_list = zeros(ncxu,N);
Ahat_list = zeros(nx,nx,N);
Rhat_list = zeros(nx,nx,N);
Qhat_list = zeros(nx,nx,N);
xn_0_hat_list = zeros(nx,N);
un_0_hat_list = zeros(nx,N);


for i=1:N
    % Dhat = [D*R^-1D^T]^-1   equation 18
    if (ismember(i,Cxu))
        Dhat_list(:,:,i) = inv(D_list(:,:,i)*inv(R)*D_list(:,:,i)');
        E_list(:,:,i) =  C_list(:,:,i);
        rhat_list(:,i) = r_list(:,i);
    else
        Dhat_list(:,:,i) = zeros(ncxu, ncxu);
        E_list(:,:,i) =  C_list(:,:,i);
        rhat_list(:,i) = r_list(:,i);
    end
    
    % equation 19
    Ahat_list(:,:,i) = A_list(:,:,i) ...
        - B_list(:,:,i)*inv(R)*D_list(:,:,i)'*Dhat_list(:,:,i)*C_list(:,:,i);
    
    Rhat_list(:,:,i) = B_list(:,:,i)*inv(R)* ...
        (eye(nu) - D_list(:,:,i)'*Dhat_list(:,:,i)*D_list(:,:,i)*inv(R))*B_list(:,:,i)';
    Qhat_list(:,:,i) = Q + C_list(:,:,i)'*Dhat_list(:,:,i)*C_list(:,:,i);
    xn_0_hat_list(:,i) = C_list(:,:,i)'*Dhat_list(:,:,i)*rhat_list(:,i);
    un_0_hat_list(:,i) = -B_list(:,:,i)*inv(R)*D_list(:,:,i)'*Dhat_list(:,:,i)*rhat_list(:,i);
end

% 3. solve backward with P(N) = Q(N)
P_list = zeros(nx,nx,N+1);  % same as Q list
z_list = zeros(nx,N+1);   % same as x_0_list
M_list = zeros(nx,nx,N);
P_list(:,:,N+1) = Qf;
z_list(:,N+1) = -Qf*xN;  

for n = N:-1:1
    % equation 50
    M_list(:,:,n) = inv(eye(nx) + Rhat_list(:,:,i)*P_list(:,:,n+1));
    % equation 51
    P_list(:,:,n) = Qhat_list(:,:,i) + Ahat_list(:,:,i)'*P_list(:,:,n+1)*M_list(:,:,n)*Ahat_list(:,:,i);
    % equation 52
    z_list(:,n) = Ahat_list(:,:,i)'*M_list(:,:,n)'*z_list(:,n+1) ...
        + Ahat_list(:,:,i)'*P_list(:,:,n+1)*M_list(:,:,n)*un_0_hat_list(:,i) ...
        + xn_0_hat_list(:,i);
end

% 4. solve over all state constraints 
% TODO: preallocate memory
Gamma_list = {}; % cell array, every element is zeros(ncx,nx,k+1)
y_list = {};
F_list = cell(length(Cx),length(Cx)); % 2d cell array

for i=1:length(Cx)
    k = Cx(i);   % constraint time index
    Gamma_k = zeros(ncx,nx,k+1); % very cofusing, because we want to store 0 to k but matlab starts index from 1
    Gamma_k(:,:,k+1) = G_list(:,:,k); % notice index of G_list
    y_k = zeros(ncx,k+1);
    y_k(:,k+1) = zeros(ncx,1);
    for n = k:-1:1
        % euqation 53
        Gamma_k(:,:,n) = Gamma_k(:,:,n+1)*M_list(:,:,n)*Ahat_list(:,:,n);
        % equation 54
        y_k(:,n) = y_k(:,n+1) + Gamma_k(:,:,n+1)*M_list(:,:,n) ...
            *(un_0_hat_list(:,n) - Rhat_list(:,:,n)*z_list(:,n+1));
    end
    
    Gamma_list{i} = Gamma_k;
    y_list{i} = y_k;
end
for i=1:length(Cx)
    k = Cx(i);
    for l=1:length(Cx)
        j = Cx(l);
        dim = min(k,j);
        F_list{i,l} = zeros(ncx,ncx,length(Cx)+1);
        if (j < k)
            F_list{i,l}(:,:,k+1) = zeros(ncx,ncx);
        else
            F_list{i,l}(:,:,j+1) = zeros(ncx,ncx);
        end
        for n = dim:-1:1
            F_list{i,l}(:,:,n) = F_list{i,l}(:,:,n+1) ...
               - Gamma_list{i}(:,:,n+1)*M_list(:,:,n)*Rhat_list(:,:,n)*Gamma_list{l}(:,:,n+1)';
        end
    end
end

% 5. Compute the Lagrangian Multipliers
% equation 46
F = sparse(ncx*length(Cx),ncx*length(Cx));
for i=1:length(Cx)
    for l=1:length(Cx)
        subF = F_list{i,l}(:,:,1);
        F((i-1)*ncx+1:(i-1)*ncx+ncx,(l-1)*ncx+1:(l-1)*ncx+ncx) = subF;
    end
end
Gamma = sparse(ncx*length(Cx),nx);
for i=1:length(Cx)
    Gamma((i-1)*ncx+1:(i-1)*ncx+ncx,:) = Gamma_list{i}(:,:,1);
end
y = zeros(ncx*length(Cx),1);
for i=1:length(Cx)
    y((i-1)*ncx+1:(i-1)*ncx+ncx) = y_list{i}(:,1);
end
% notice the index of h_list is different
H = zeros(ncx*length(Cx),1);
for i=1:length(Cx)
    H((i-1)*ncx+1:(i-1)*ncx+ncx) = h_list(:,Cx(i));
end

% v size ncx*length(Cx) , equation 56
v = (-F)\(y+H);

% equation 57
s_list = zeros(nx,N+1);
for i =1:N+1
    s_list(:,i) = z_list(:,i);
    for l=1:length(Cx)
        if Cx(l) >= i
            s_list(:,i) = s_list(:,i) +  Gamma_list{l}(:,:,i)'*v((l-1)*ncx+1:(l-1)*ncx+ncx);
        end
    end
end

% list constain solution
u_list = zeros(nu,N);  % from 0 to N-1
u_ff = zeros(nu,N);  % from 0 to N-1
K = zeros(nu,nx,N);  % from 0 to N-1
x_list = zeros(nx,N+1);  % from 0 to N
% solve forward
for n =1:N
    % euqation 58
    ve = M_list(:,:,n)*(un_0_hat_list(:,n)-Rhat_list(:,:,n)*s_list(:,n+1));
    % equation 59
    x_list(:,n+1) = M_list(:,:,n)*Ahat_list(:,:,n)*x_list(:,n) + ve;
    % equation 60
    lambda = P_list(:,:,n+1)*x_list(:,n+1) + s_list(:,n+1);
    
    if (ismember(n, Cxu))
        miu = Dhat_list(:,:,n)*(E_list(:,:,n)*x_list(:,n) ...
            - D_list(:,:,n)*inv(R)*B_list(:,:,n)'*lambda + rhat_list(:,n));
    else
        miu = zeros(ncxu, 1);
    end
    
    
    Dhat = Dhat_list(:,:,n);
    D = D_list(:,:,n);
    A = A_list(:,:,n);
    Ahat = Ahat_list(:,:,n);
    B = B_list(:,:,n);
    C = C_list(:,:,n);
    D = D_list(:,:,n);
    M = M_list(:,:,n);
    P = P_list(:,:,n+1);
    rhat = rhat_list(:,n);
    s = s_list(:,n+1);
    u_list(:,n) = -inv(R)*(B'*lambda + D'*miu);
    
    
    if (ismember(n, Cxu))
        K(:,:,n) = -inv(R)*B'*P*M*Ahat-inv(R)*D'*Dhat*C+inv(R)*D'*Dhat*D*inv(R)*B'*P*M*Ahat;
        u_ff(:,n) = -inv(R)*B'*P*ve - inv(R)*B'*s ...
                    +inv(R)*D'*Dhat*D*inv(R)*B'*P*ve ...
                    +inv(R)*D'*Dhat*D*inv(R)*B'*s ...
                    -inv(R)*D'*Dhat*rhat;
    else
         K(:,:,n) = -inv(R)*B'*P*M*Ahat;
         u_ff(:,n) = -inv(R)*B'*P*ve - inv(R)*B'*s;
    end
end

nSoln = N;
Soln(nSoln+1).K = zeros(nx,nu);
Soln(nSoln+1).uff = zeros(nu,1);
Soln(nSoln+1).x = zeros(nx,1);

for idx=1:nSoln
    i = nSoln-idx+1;
    Soln(i).K = K(:,:,i);
    Soln(i).uff = u_ff(:,i);
    Soln(i).x = x_list(:,i);
end
Soln(N+1).x = x_list(:,N+1);

end

