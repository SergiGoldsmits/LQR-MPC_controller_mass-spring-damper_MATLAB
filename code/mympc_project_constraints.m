% Constrained Model Predictive Control Function
% A: A matrix of the linear considered dynamic system
% B: B matrix of the linear considered dynamic system
% Q: status weight into the MPC cost function
% R: inputs weight into the MPC cost function
% S: final state weight (prediction horizon instant time) into the MPC cost function
% N: prediction horizon
% umin: inputs lower limit (scalar)
% umax: inputs upper limit (scalar)
% X: measured status at the current instant time
function u = mympc_project_constraints(Az,Bz,Q_LQ,R_LQ,S,N,umin,umax,Ubar_z,xmin,xmax,Xbar_z,x)

m=size(Bz,2);
% n=size(Az,1);

% Q matrix for Open-Loop MPC (with Q)
Qsig = blkdiag(kron(eye(N-1),Q_LQ),S);
Rsig = kron(eye(N),R_LQ);

% A matrix
Asig = Az;
for i = 2:N
    Asig = [Asig; Az^i];
end

% B matrix
Bsig = [];
temp = [];
for i = 1:N
    temp = zeros(size(Bz,1)*(i-1),1);
    for j = 0:N-i
        temp = [temp; Az^(j)*Bz];
    end
    Bsig = [Bsig temp];
end

% H,F 
H = Bsig'*Qsig*Bsig + Rsig;
F = Asig'*Qsig*Bsig;
ft = x'*F;
f=ft';

%input and status constraints definition
lb_u = [repmat(umin, N*m,1)];
ub_u = [repmat(umax, N*m,1)];

lb_x = repmat(xmin, N, 1);
ub_x = repmat(xmax, N, 1);

Aineq = [ Bsig; -Bsig ];
Bineq = [ ub_x - Asig * x;
         -lb_x + Asig * x ];


options = optimset('Algorithm', 'interior-point-convex','Diagnostics','off', ...
    'Display','off');

%solve the quadratic programming problem
% U = quadprog(H,f,Aineq,Bineq,[],[],lb_u,ub_u,x0,options);
[U,fval,exitflag] = ...
quadprog(H,f,Aineq,Bineq,[],[],lb_u,ub_u,[],options);

if exitflag ~= 1
    disp('error', exitflag)
end

%get the optimal input value (the receding horizon principle is applied)
u = U(1:m);

end
