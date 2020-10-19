function Soln = ecLQR_laine(param, xN,  A_list, B_list, C_list, D_list, G_list, r_list, h_list)
%ECLQR_SIDERIS solve LQR with equality constraint using sideris method
%              in paper Efficient computation of feedback control for equality-constrained lqr
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
ncxu = param.ncxu;    %lt
ncx = param.ncx;      %lT

Cxu = param.Cxu;
Cx  = param.Cx;
Q = param.Q;
Qf = param.Qf;
R = param.R;

% convert Gx=h into Cx+Du=r format for Laine
C_list = [C_list; G_list];
D_list = [D_list; zeros(ncx, nu, N)];
r_list = [r_list; h_list];

% 2. init cost to go and constraint to go
% use notations in paper 
% cost to go is 0.5*x'*VxxT*x + vxlT*x 
% constraint to go is HxT and hlT
% 2.1 equation 9
VxxT = Qf;
vxlT = -Qf*xN;
HxT = G_list(:,:,N);
hlT = h_list(:,N);

% cost to go and costraint to go during iteration
Vxxt1 = VxxT;
vxlt1 = vxlT;
Hxt1 = HxT;
hlt1 = hlT;

k_list = zeros(nu,N);  % from 0 to N-1
K_list = zeros(nu,nx,N);  % from 0 to N-1

for i=N:-1:1
    % necessary varialbe conversion
    qxlt = zeros(nx,1);
    qult = zeros(nu,1);
    flt = zeros(nx,1);
    Fxt = A_list(:,:,i);
    Fut = B_list(:,:,i);
    Qxxt = Q;
    Quut = R;
    Quxt = zeros(nu,nx);
    Gxt = C_list(:,:,i);
    Gut = D_list(:,:,i);
    gl = r_list(:,i);
    % equation 12
    mxlt = qxlt + Fxt'*vxlt1;
    mult = qult + Fut'*vxlt1;
    Mxxt = Qxxt + Fxt'*Vxxt1*Fxt;
    Muut = Quut + Fut'*Vxxt1*Fut;
    Muxt = Quxt + Fut'*Vxxt1*Fxt;
    
    Nxt = [Gxt;
           Hxt1*Fxt];
    Nut = [Gut;
           Hxt1*Fut]; 
    nlt = [gl;
           Hxt1*flt+hlt1];
    % svd to get P and Z, equation 13d
    [U,S,V] = svd(Nut);
    rankNut = nnz(S);
    I = eye(size(Nxt,1));
    
%     % equation 14 and 15
%     y = -pinv(Nut*P)*(Nxt*xt+nlt);
%     w = -inv(Z'*Muut*Z)*Z'*(Muxt*xt+mult);
    if (rankNut == 0)
        Z = V;
        % equation 17 and 18
        K = -( Z*(Z'*Muut*Z)\Z'*Muxt );
        k = -( Z*(Z'*Muut*Z)\Z'*mult );
        k_list(:,i) = k;
        K_list(:,:,i) = K;
    elseif (rankNut == nu)
        P = V;
        % equation 17 and 18
        K = -( P*pinv(Nut*P)*Nxt);
        k = -( P*pinv(Nut*P)*nlt);
        k_list(:,i) = k;
        K_list(:,:,i) = K;
    else
        P = V(1:rankNut,:);
        Z = V(rankNut+1:nu,:);
        % equation 17 and 18
        K = -( P*pinv(Nut*P)*Nxt + Z*(Z'*Muut*Z)\Z'*Muxt );
        k = -( P*pinv(Nut*P)*nlt + Z*(Z'*Muut*Z)\Z'*mult );
        k_list(:,i) = k;
        K_list(:,:,i) = K;
    end
    % remove redudant terms, the paragraph below equation 21
    c = [hlt1 Hxt1];
    [U,S,V] = svd(c);
    c = U'*c;
    rows = size(c,1);
    c = c(1:rows-rankNut,:);
    hlt1 = c(:,1);
    Hxt1 = c(:,2:end);

    % update constraint to go , equation 20 and 21
    Hxt = Nxt + Nut*K;
    hlt = nlt + Nut*k;
    % update cost to go, equation 24 and 25
    Vxxt = Mxxt + 2*K'*Muxt + K'*Muut*K;
%     Vxxt = Mxxt + 2*Muxt'*K + K'*Muut*K;
    Vxxt = (Vxxt + Vxxt.')/2;   % this is an astonishingly important step, I debugged almost a day to figure out
    vxlt = mxlt + K'*mult + (Muxt'+K'*Muut)*k;
    
    % update 
    Hxt1 = Hxt;
    hlt1 = hlt;
    Vxxt1 = Vxxt;
    vxlt1 = vxlt;
end

% a final forward pass to get x_list
x_list = zeros(nx,N+1);  % from 0 to N
x_list(:,1) = param.x0;
for i=1:N
    u = K_list(:,:,i)*x_list(:,i) + k_list(:,i);
    
    x_list(:,i+1) = A_list(:,:,i)*x_list(:,i) + B_list(:,:,i)*u;
end

nSoln = N;
Soln(nSoln+1).K = zeros(nx,nu);
Soln(nSoln+1).k = zeros(nu,1);
Soln(nSoln+1).x = zeros(nx,1);

for i=1:nSoln
    Soln(i).K = K_list(:,:,i);
    Soln(i).k = k_list(:,i);
    Soln(i).x = x_list(:,i);
end
Soln(N+1).x = x_list(:,N+1);

end