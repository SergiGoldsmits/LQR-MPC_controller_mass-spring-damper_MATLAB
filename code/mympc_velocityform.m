function u = mympc_velocityform(A_tilde, B_tilde, QMPC, R_LQ, S_tilde, N, umin, umax,x)


m = size(B_tilde, 2); 
n = size(A_tilde, 1); 



Qsig = blkdiag(kron(eye(N-1), QMPC), S_tilde);
Rsig = kron(eye(N), R_LQ);


Asig = A_tilde;
for i = 2:N
    Asig = [Asig; A_tilde^i];
end


Bsig = zeros(N*n, N*m);
for i = 1:N
    for j = 1:i
        Bsig(n*(i-1)+1:n*i, m*(j-1)+1:m*j) = A_tilde^(i-j) * B_tilde;
    end
end


% Funzione costo
H = Bsig'*Qsig*Bsig + Rsig;
F = Asig'*Qsig*Bsig;
ft = x'*F;
f=ft';

% Vincoli su Δu
lb_du = repmat(umin , N, 1);
ub_du = repmat(umax , N, 1);




% Opzioni per quadprog
options = optimset('Algorithm', 'interior-point-convex', ...
                   'Display', 'off');
[U,fval,exitflag] = ...
quadprog(H,f,[],[],[],[],lb_du,ub_du,[],options);


% Ricostruisci u(k) = u(k-1) + Δu(1)
u = U(1:m);

end
