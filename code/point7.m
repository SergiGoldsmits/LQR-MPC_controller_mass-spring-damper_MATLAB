
%SYSTEM DEFINITION

L = 6;   % number of masses        
n = 2*L; %dimension (n of states)         
M = 1; % mass
k_bis = 8; % spring constant
h = 2; %damping constant

%initialization of matrices
A_bis = zeros(n);  %nxn  when n: n. of states
B = zeros(n,1); %n*m   when m: n. of inputs
C = zeros(1,n); %p*n   when p: n. of outputs
D = 0; %p*m (no u in y)

% position equations: z_dot = v
for j = 1:L
    A_bis(2*j - 1, 2*j) = 1;
end

% velocity equations

% mass 1
A_bis(2,1) = -2*k_bis/M;
A_bis(2,2) = -2*h/M;
A_bis(2,3) =  k_bis/M;
A_bis(2,4) =  h/M;

% central masses (2->5)
for j = 2:L-1
    i = 2*j;
    A_bis(i, i-3) =  k_bis/M;
    A_bis(i, i-2) =  h/M;
    A_bis(i, i-1) = -2*k_bis/M;
    A_bis(i, i)   = -2*h/M;
    A_bis(i, i+1) =  k_bis/M;
    A_bis(i, i+2) =  h/M;
end

% mass 6
A_bis(12,9)  =  k_bis/M;
A_bis(12,10) =  h/M;
A_bis(12,11) = -k_bis/M;
A_bis(12,12) = -h/M;
B(12) = 1/M;

% Output
C(1) = 1;  %where z1=1

% state-space sys
sys_bis = ss(A_bis, B, C, D);

%system's transfer function CONTINOUS TIME
Gs_bis = tf(sys_bis);

%system's transfer function DISCRETE TIME
Ts=0.1;  %sampling time
sys_d_bis = c2d(sys_bis, Ts);     
[Az_bis, Bz_bis, Cz, Dz] = ssdata(sys_d_bis);
Gz = tf(sys_d_bis);
