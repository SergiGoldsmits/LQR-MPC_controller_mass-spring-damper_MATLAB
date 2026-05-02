clear
close all
clc
warning off

%SYSTEM DEFINITION

L = 6;   % number of masses        
n = 2*L; %dimension (n of states)         
M = 1; % mass
k = 10; % spring constant
h = 2; %damping constant

%initialization of matrices
A = zeros(n);  %nxn  when n: n. of states
B = zeros(n,1); %n*m   when m: n. of inputs
C = zeros(1,n); %p*n   when p: n. of outputs
D = 0; %p*m (no u in y)

% position equations: z_dot = v
for j = 1:L
    A(2*j - 1, 2*j) = 1;
end

% velocity equations

% mass 1
A(2,1) = -2*k/M;
A(2,2) = -2*h/M;
A(2,3) =  k/M;
A(2,4) =  h/M;

% central masses (2->5)
for j = 2:L-1
    i = 2*j;
    A(i, i-3) =  k/M;
    A(i, i-2) =  h/M;
    A(i, i-1) = -2*k/M;
    A(i, i)   = -2*h/M;
    A(i, i+1) =  k/M;
    A(i, i+2) =  h/M;
end

% mass 6
A(12,9)  =  k/M;
A(12,10) =  h/M;
A(12,11) = -k/M;
A(12,12) = -h/M;
B(12) = 1/M;

% Output
C(1) = 1;  %where z1=1

% state-space sys
sys = ss(A, B, C, D);

%system's transfer function CONTINOUS TIME
Gs = tf(sys)
pzmap(Gs)
poles_G=pzmap(Gs)
zeros_G=tzero(Gs)

%system's transfer function DISCRETE TIME
Ts=0.1;  %sampling time
sys_d = c2d(sys, Ts);  
[Az, Bz, Cz, Dz] = ssdata(sys_d);
Gz = tf(sys_d);




%variables
tsim=15;
  % x0 = [9,0,9,0,9,0,9,0,9,0,9,0];
% x0 = [9,0,0,0,0,0,0,0,0,0,0,0];
 x0 = [3,0,3,0,3,0,3,0,3,0,3,0];

% input constraints
umin=-15;
umax=+15;

% states constraints
xmin = [-10; -Inf; -10; -Inf; -10; -Inf; -10; -Inf; -10; -Inf; -10; -Inf];
xmax = [ 10;  Inf;  10;  Inf;  10;  Inf;  10;  Inf;  10;  Inf;  10;  Inf];



%equilibrium 
Ybar=1;

%continous eq
Ubar_s=inv(dcgain(Gs))*Ybar;
Xbar_s = -A \ (B * Ubar_s);

%discrete eq
Ubar_z=inv(dcgain(Gz))*Ybar;
Xbar_z = (eye(12) - Az) \ (Bz * Ubar_z);


%point 1: LQ design

C_state=eye(12);
D_state=zeros(12, 1);

q = 10e-5; 
r = 10e-6;
Q_LQ = q * eye(12);   
Q_LQ(1,1) = 1 + q;
R_LQ = r;

K_s=lqr(A,B,Q_LQ,R_LQ);   %LQ continous
[K_z,Pr,E]=dlqr(Az,Bz,Q_LQ,R_LQ);  %LQ discrete
% S = Pr;
 S = zeros(n);


feedbackSystem = Az-Bz*K_z;  %stability check
poles_feedback=eig(feedbackSystem)

%point 2: MPC design

%%POINT 6

B_noise=[Bz ones(size(Bz,1),1)];
D_noise=[D_state zeros(size(D_state,1),1)];


power_vx=0.00001;
power_vy=0.000001;

% set Kalman filter parameters
QK=q* eye(12); % variance of v_x
RK=r;            % variance of v_y

 
NK=0;               % covariance v_x,v_y

x0K=1.5*x0;         %initial guess for x0

%POINT 7
point7;
R_tilde=R_LQ*100;
dQ=[q+1,repmat(q,1,11),q+1];
Q_tilde= diag(dQ);
QMPC=Q_tilde;
S_tilde=zeros(13);
num=size(Az,1);
A_tilde=[Az, zeros(num,1);
        -Cz*Az , 1];
B_tilde=[Bz; -Cz*Bz];






