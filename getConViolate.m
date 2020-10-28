function vio = getConViolate(N, param, x_list, u_list, A_list, B_list, C_list, D_list, G_list, r_list, h_list, S_list)
vio = 0;
if nargin < 12
    error('please pass in A and B to `getConViolate`');
end
if nargin < 13
    S_list = {}; % no cross-timestep constraints
end

for i=1:N
    % dynamics constraints
    term = A_list(:, :, i)*x_list(:, i) + B_list(:, :, i)*u_list(:, i) - x_list(:, i+1);
    vio = vio + term'*term;
    
    % joint state/control constraints
    if (ismember(i,param.Cxu))
        term = C_list(:,:,i)*x_list(:,i) + D_list(:,:,i)*u_list(:,i) + r_list(:,i);
        vio = vio + term'*term;
    end
    
    % state-only constraints
    if (ismember(i,param.Cx))
        term = G_list(:,:,i)*x_list(:,i) + h_list(:,i);
        vio = vio + term'*term;
    end
%     fprintf('%d: %e\n', i, vio);
end

% final state constraints
if (ismember(N+1,param.Cx))
    term = G_list(:,:,N+1)*x_list(:,N+1) + h_list(:,N+1);
    vio = vio + term'*term;
end

% xN constraint
term = x_list(:, N+1) - param.xN;
vio = vio + term'*term;

% cross-timestep constraints
for S = S_list
    S = S{1};
    lhs = 0;
    for xi = 1:length(S.xn)
        lhs = lhs + S.xmat(:, :, xi) * x_list(:, S.xn(xi));
    end
    for ui = 1:length(S.un)
        lhs = lhs + S.umat(:, :, ui) * u_list(:, S.un(ui));
    end
    vio = (lhs - rhs)' * (lhs - rhs);
end

end