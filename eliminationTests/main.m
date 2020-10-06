%%  Gerry Chen
%   main.m - compares different elimination methods with mixed constraint
%   models
%   Created:        Sept 24, 2020
%   Last modified:  Oct 6, 2020

clear;

%% Problem setup
% solve min || Hx - z ||^2
%       s.t. Ax - b = 0

n = 5;      % state dimension
nmeas = 7;  % # of measurements (too many measurements)
nconst = 0; % # of constraints (not fully constrained)

n = 3;
nmeas = 11 - 2;
nconst = 2;

H = rand(nmeas, n);
z = zeros(nmeas, 1);
z = rand(nmeas, 1);
A = rand(nconst, n); % A(end, 1:4) = 0; A(1, 2:end) = 0;
% A(1, 2:5) = 0;
b = rand(nconst, 1);

A = [1, -1, 0; 0, 1, -1]; b = [0; 1];
z = z.*0;
J = [A, b; H, z]
noisematrix = [zeros(nconst, 1); ones(nmeas, 1)]

% H =  [-1.,  0.,  1.,  0.,  0.,  0., -0.2;...
%       0., -1.,  0.,  1.,  0.,  0.,  0.3;...
%       1.,  0.,  0.,  0., -1.,  0.,  0.2;...
%       0.,  1.,  0.,  0.,  0., -1., -0.1];
% H = [...
%       1,0,0,0,0,1,  0;... // u+z = 0
%       0,0,0,0,1,0,  0;... // y^2
%       0,0,1,1,0,0,  0;... // w+x = 0
%       0,1,0,0,0,0,  0;... // v^2
%       0,0,0,0,0,1,  0];  %// z^2
% Ab = [...
%       1,0,0,  0;... //
%       0,1,0,  0;... //
%       0,0,1,  0;... //
%      -1,0,1,  0;... // x=z
%       1,0,0,  0;... //
%       0,1,0,  0;... //
%       0,0,1,  0;... //
%      0,-1,1,  0;... // y=z
%       1,0,0,  0;... //
%       0,1,0,  0;... //
%       0,0,1,  0];% //
% H = H(:, 1:end-1);
% z = H(:, end);
% 
% % version 3
% Ab = [...
%       1,0,0,  0;... //
%       0,1,0,  0;... //
%       0,0,1,  0;... //
%      -1,0,1,  0;... // x=z
%       1,0,0,  0;... //
%       0,1,0,  0;... //
%       0,0,1,  0;... //
%      0,-1,1,  0;... // y=z
%       1,0,0,  0;... //
%       0,1,0,  0;... //
%       0,0,1,  0];% //
% noisematrix = ones(size(Ab, 1), 1);
% noisematrix(4) = 0;
% noisematrix(8) = 0;

A = J(noisematrix == 0, 1:end-1);
b = J(noisematrix == 0, end);
H = J(noisematrix ~= 0, 1:end-1);
z = J(noisematrix ~= 0, end);

% util
timesinf0 = @(a, b) a(~isinf(a)) .* b(~isinf(a)) + any(b(isinf(a)) > 1e-9)*1e50;
error = @(Ab, prec, x) sum(timesinf0(prec, (Ab * [x;-1]).^2));
buffer1 = @(M, ref) [M; ones(size(ref, 1)-size(M, 1), size(M, 2))];
buffer0 = @(M, ref) [M; zeros(size(ref, 1)-size(M, 1), size(M, 2))];
Rdscale = @(Rd, NM) Rd ./ buffer1(sandpaper(1./NM), Rd); % returns Rd normalized to NM

%% initial "guess" (dummy value)
NS = null(A);
x1 = A \ b + NS * rand(size(NS, 2), 1); % a dummy value
x1err = error(J, 1./noisematrix.^2, x1);

%% real sol: quadprog + qr
fprintf('Computing truth...\t');
[xsol, fval, ~, ~] = quadprog(H'*H, H'*z, [], [], A, b, [], [], [],...
                              optimoptions('quadprog', 'Display', 'off'));
[~, Rdexpected] = qr(J.*sandpaper(1./noisematrix));
assert( abs(Rdexpected(n+1, n+1)^2 / 2 - fval) < 1e-6, 'offset error not correct' );
fprintf('Truth computed\n');

%% unit test utils
checkR = @(Rd, NM) checkR_(Rdexpected, Rd, NM, Rdscale);
checkX1err = @(Rd, NM) checkX1err_(x1, x1err, Rdexpected, Rd, NM, error, n);
checkXsol = @(Rd) assert(all(abs(...
    xsol - Rd(:, 1:end-1) \ Rd(:, end)...
        ) < 1e-9), 'solution from Rd not correct');

%% gram-schmidt
[Rd_GS, Q_GS, NM_GS] = gramschmidt(J, noisematrix);
fprintf('Checking GS...     \t');
checkR(Rd_GS, NM_GS);
checkX1err(Rd_GS, NM_GS);
checkXsol(Rd_GS);
fprintf('Gram-Schmidt correct!\n');

%% double QR
[Rd_dQR, NM_dQR] = doubleQR(J, noisematrix);
fprintf('Checking double QR...\t');
checkR(Rd_dQR, NM_dQR);
checkX1err(Rd_dQR, NM_dQR);
checkXsol(Rd_dQR);
fprintf('double QR correct!\n');

%% SVD
% TODO
% [Rd_SVD, NM_SVD] = SVDelim(J, noisematrix);
% fprintf('Checking SVD...\t');
% checkR(Rd_SVD, NM_SVD);
% checkX1err(Rd_SVD, NM_SVD);
% checkXsol(Rd_SVD);
% fprintf('SVD correct!\n');


%% funcs
function [v] = sandpaper(v)
    v(isinf(v)) = 1e6;
end
function [] = checkR_(Rdexp, Rd, NM, Rdscale)
    Rdexp = Rdscale(Rdexp, NM);
    assert((size(Rd, 1) == (size(Rd, 2)-1)) || ...
           (size(Rd, 1) == size(Rd, 2)), 'Rd wrong dimension');
    for rowi = 1:size(Rd, 1)
        assert(all(abs(Rd(rowi, :) - Rdexp(rowi, :)) < 1e-6) ||... % +/-
               all(abs(Rd(rowi, :) + Rdexp(rowi, :)) < 1e-6),...
               'Rd doesn''t match');
    end
end
function [] = checkX1err_(x1, x1errExp, Rdexp, Rd, NM, error, n)
    if (size(Rd, 1) == (size(Rd, 2) - 1))
        x1err = error(Rd, 1./NM.^2, x1) + Rdexp(n+1, n+1).^2; % off by a constant
    else
        x1err = error(Rd, 1./NM.^2, x1);
    end
    assert(all(abs(x1errExp - x1err) < 1e-6), 'x1 error from Rd not correct');
end
function [Rd, Q, newNoisematrix] = gramschmidt(Ab, noisematrix)
    Rd = zeros(0, size(Ab, 2));
    Q = zeros(size(Ab, 1), 0);
    newNoisematrix = zeros(0, 1);
    constraintrows = ~noisematrix;
    m = size(Ab, 1);
    % order of columns
    constraintrows = find(noisematrix < 1e-9);
    constraintcols = [];
    for row = constraintrows'
        constraintcols(end+1) = find(Ab(row, :) > 1e-9, 1);
    end
%     colorder = [constraintcols, setdiff(1:(size(Ab, 2)-1), constraintcols)];
    colorder = 1:(size(Ab, 2)-1);
%     colorder = 1:(size(Ab, 2));
    % loop
    for col = colorder % loop over columns
        a = Ab(1:m, col);
        constraintrows = (noisematrix < 1e-9) .* (abs(a) > 1e-9);
        if any(constraintrows)
            row = find(constraintrows, 1);
%             Rd(end+1, :) = Ab(row, :) * sum(Ab(:, col).^2) / Ab(row, col);
            Rd(end+1, :) = Ab(row, :);
            Q(:, end+1) = Ab(:, col) / sum(Ab(:, col).^2);
            newNoisematrix(end+1, 1) = 0;
            m = m - 1;
            Ab(row, :) = Ab(m + 1, :);
            noisematrix(row) = noisematrix(m + 1);
            noisematrix(m + 1) = [];
            areduced = Ab(1:m, col) / Rd(end, col);
            Ab(1:m, col+1:end) = Ab(1:m, col+1:end) - areduced * Rd(end, col+1:end);
        else
            pseudo = a / sqrt(sum(a.^2));
            Q(1:m, end+1) = pseudo;
%             Rd(end+1, col) = sqrt;
            Rd(end+1, col:end) = pseudo' * Ab(1:m, col:end) / sqrt(sum(a.^2));
            Ab(1:m, col+1:end) = Ab(1:m, col+1:end) - pseudo .* Rd(end, col+1:end);
            newNoisematrix(end+1, 1) = 1 ./ sqrt(sum(a.^2));
%             newNoisematrix(end+1, 1) = 1;
        end
%         Ab
%         Q
%         Rd
    end
%     newNoisematrix = ones(size(newNoisematrix));
%     Q*Rd
%     Ab
%     Q(end, 2:end) = Q(1, 2:end);
%     Q(1, 2:end) = 0;
%     Q(end-1, 3:end) = Q(2, 3:end);
%     Q(2, 3:end) = 0;
end
function [Rd, NM] = doubleQR(J, noisematrix)
    n = size(J, 2)-1;
    nconst = sum(noisematrix == 0);
    Ab = J(noisematrix == 0, :);
    Hz = J(noisematrix ~= 0, :);
    [~, Rc] = qr(Ab);
    [Hz2, Ccols] = subinto(Rc, Hz);
    [Qh, Rh] = qr(Hz2);
    Rd = zeros(n+1);
    Rd(1:nconst, :) = Rc;
    Rd(nconst+1:end, all((1:size(Rc, 2))~=Ccols', 1)) = Rh(1:n+1-nconst, :);
    NM = [zeros(nconst, 1); ones(n+1-nconst, 1)];
end
function [H, Rcols] = subinto(R, H)
    Rcols = [];
    for row = R'
        leading = find(row ~= 0, 1);
        Rcols(end+1) = leading;
        H = H - H(:, leading) .* row';
    end
    H(:, Rcols) = [];
end
function [Rd, NM] = SVDelim(J, noisematrix)
    Ab = J(noisematrix == 0, :);
    Hz = J(noisematrix ~= 0, :);
    Nut = Ab(:, 1:end-1);
    [U, S, V] = svd(Nut);
    rankS = sum(any(S));
    Pyt = V(:, 1:rankS);
    Zwt = V(:, rankS+1:end); % null space
    pinv1 = pinv(Nut * Pyt); % TODO: use svd
    Hnew = Hz(:, 1:end-1) * Zwt;
    wt = Hz(:, end) \ Hnew;
end