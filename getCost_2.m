function J = getCost_2(N,x,u,Q, R, Qf, xN)
J = 0;
for i=1:N
    state = x(:,i);
    ctrl = u(:,i);
    J = J + state'*Q*state + ctrl'*R*ctrl;
end
N = N+1;
J = J + ([x(:,N)]-xN)'*Qf*([x(:,N)]-xN);
end