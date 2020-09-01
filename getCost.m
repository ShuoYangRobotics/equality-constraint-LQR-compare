function J = getCost(N,x,y,u,Q, R, F, xN)
J = 0;
for i=1:N-1
    state = [x(i);y(i)];
    ctrl = u(:,i);
    J = J + state'*Q*state + ctrl'*R*ctrl;
end
J = J + ([x(N);y(N)]-xN)'*F*([x(N);y(N)]-xN);
end