function J = getCost(N,x,y,u,Q, R, Qf, xN)
J = 0;
for i=1:N
    state = [x(i);y(i)];
    ctrl = u(:,i);
    J = J + state'*Q*state + ctrl'*R*ctrl;
end
N = N+1;
J = J + ([x(N);y(N)]-xN)'*Qf*([x(N);y(N)]-xN);
end