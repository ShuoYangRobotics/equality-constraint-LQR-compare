function [R, D, J] = partialQR(M, n)
%partialQR(M, n) calculates the partial QR factorization of M on n columns.
%   M = partialQR(M, n) returns the partially factored M matrix
%   [R, D, J] = partialQR(M, n) returns:
%       R - the upper triangular matrix on the first n columns,
%       D - the bayes arrow
%       J - the remaining joint factor on the separator
%       ie.  M = Q * [R, D; 0, J]
    [Q1, M(:, 1:n)] = qr(M(:, 1:n));
    M(:, (n+1):end) = Q1'*M(:, (n+1):end);
    if nargout < 2
        R = M;
    else
        R = M(1:n, 1:n);
        D = M(1:n, (n+1):end);
        J = M((n+1):end, (n+1):end);
    end
end