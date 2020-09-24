%%  Gerry Chen
%   main.m - compares different elimination methods
%   Sept 24, 2020

clear;

%% Problem setup
% solve min || Hx - z ||^2
%       s.t. Ax - b = 0

n = 5;      % state dimension
nmeas = 7;  % # of measurements (too many measurements)
nconst = 0; % # of constraints (not fully constrained)

H = rand(nmeas, n);
z = zeros(nmeas, 1);
z = rand(nmeas, 1);
A = rand(nconst, n); % A(end, 1:4) = 0; A(1, 2:end) = 0;
% A(1, 2:5) = 0;
b = rand(nconst, 1);

NS = null(A);
x = A \ b + NS * rand(size(NS, 2), 1);

noisematrix = [zeros(nconst, 1); ones(nmeas, 1)];
Ab = [A, b; H, z]
timesinf0 = @(a, b) a(~isinf(a)) .* b(~isinf(a)) + any(b(isinf(a)) > 1e-9)*1e50;
error = @(Ab, prec, x) sum(timesinf0(prec, (Ab * [x;-1]).^2));
fprintf('baseline error: %f\n', error(Ab, noisematrix, x));

%% gram-schmidt
[Rd, Q, RdNoisematrix] = gramschmidt(Ab, noisematrix)
fprintf('baseline error: %f\n', error(Ab, 1./noisematrix.^2, x));
fprintf('error: %f\n', error(Rd, 1./RdNoisematrix.^2, x));
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
    colorder = [constraintcols, setdiff(1:(size(Ab, 2)-1), constraintcols)];
    colorder = 1:(size(Ab, 2)-1);
    % loop
    for col = colorder % loop over columns
        a = Ab(1:m, col);
        constraintrows = (noisematrix < 1e-9) .* (abs(a) > 1e-9);
        if any(constraintrows)
            row = find(constraintrows, 1);
%             Rd(end+1, :) = Ab(row, :) * sum(Ab(:, col).^2) / Ab(row, col);
            Rd(end+1, :) = Ab(row, :);
            Q(:, end+1) = Ab(:, col) / sum(Ab(:, col).^2);
            newNoisematrix(end+1, 1) = inf;
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
            Rd(end+1, col:end) = pseudo' * Ab(1:m, col:end);
            Ab(1:m, col+1:end) = Ab(1:m, col+1:end) - pseudo .* Rd(end, col+1:end);
            newNoisematrix(end+1, 1) = sum(a.^2);
%             newNoisematrix(end+1, 1) = 1;
        end
%         Ab
%         Q
%         Rd
    end
    newNoisematrix = ones(size(newNoisematrix));
    Q*Rd
    Ab
%     Q(end, 2:end) = Q(1, 2:end);
%     Q(1, 2:end) = 0;
%     Q(end-1, 3:end) = Q(2, 3:end);
%     Q(2, 3:end) = 0;
end