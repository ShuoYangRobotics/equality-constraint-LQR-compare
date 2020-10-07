function [R, Dd, Sd, sigma] = eliminate_partial_dQR(J, sigma, nFrontal)
%eliminate_partial_dQR(J, sigma, nFrontal) performs partial elimination on
%a factor given by ||J*[x; -1]||_sigma^2 using double-QR.  Basically the
%same result as partialQR except it can also handle constrained sigmas.
%   R - upper triangular on nFrontal keys
%   Dd - Bayes arrow
%   Sd - remaining factor on joint
%   sigma - sigma vector
    n = size(J, 2)-1;
    nconst = sum(sigma == 0);
    Ab = J(sigma == 0, :);
    Hz = J(sigma ~= 0, :);
    [~, Rc] = qr(Ab);
    [Hz2, Ccols] = subinto(Rc, Hz);
    nFrontalH = nFrontal - sum(Ccols<=nFrontal);
    [Rh, D, J] = partialQR(Hz2, nFrontalH);
    R =  [Rc(:, 1:nFrontal); zeros(nFrontalH, nFrontal-nFrontalH), Rh];
    Dd = [Rc(:, (nFrontal+1):end); D];
    Sd = J;
    sigma = [zeros(nconst, 1); ones(n+1-nconst, 1)];
    if nargout < 3
        R = [R, Dd; zeros(size(Sd, 1), nFrontal), Sd];
    end
end

function [H, Rcols] = subinto(R, H)
    Rcols = [];
    for row = R'
        leading = find(row ~= 0, 1);
        Rcols(end+1) = leading;
        H = H - H(:, leading) .* row';
    end
    if nargout > 1
        H(:, Rcols) = [];
    end
end