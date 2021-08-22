% Reference paper parameters
% kf=3.13e-5
% km=7.5e-7
% m=0.0025
% Ixx=0.00104
% Iyy=0.00104
% Izz=0.0135

%Actual Hardware parameters
kf=9.8e-6  % thrust factor
km=1.6e-7  % drag factor
m=2        % mass of quadrotor
Ixx=0.0035 % Inertia along x-axis
Iyy=0.0035 % Inertia along y-axis
Izz=0.005  % Inertia along z-axis

l=0.225    % length of arm
w1=100     % max angular speed of rotor 1
w2=100     % max angular speed of rotor 2
w3=100     % max angular speed of rotor 3
w4=100     % max angular speed of rotor 4
u1=((w1^2)+(w2^2)+(w3^2)+(w4^2))*kf % thrust input to the quadrotor

% State space model: x' = Ax + Bu
% x[states] = [x y z x'y'z' phi theta zhi p q r]
A=[0 0 0 1 0 0 0 0 0 0 0 0;
   0 0 0 0 1 0 0 0 0 0 0 0;
   0 0 0 0 0 1 0 0 0 0 0 0;
   0 0 0 0 0 0 0 u1/m 0 0 0 0;
   0 0 0 0 0 0 -u1/m 0 0 0 0 0;
   0 0 0 0 0 0 0 0 0 0 0 0;
   0 0 0 0 0 0 0 0 0 1 0 0;
   0 0 0 0 0 0 0 0 0 0 1 0;
   0 0 0 0 0 0 0 0 0 0 0 1;
   0 0 0 0 0 0 0 0 0 0 0 0;
   0 0 0 0 0 0 0 0 0 0 0 0;
   0 0 0 0 0 0 0 0 0 0 0 0]
   
B=[0 0 0 0;
   0 0 0 0;
   0 0 0 0;
   0 0 0 0;
   0 0 0 0;
   1/m 0 0 0;
   0 0 0 0;
   0 0 0 0;
   0 0 0 0;
   0 01/Ixx 0 0;
   0 0 1/Iyy 0;
   0 0 0 1/Izz]
   
C=[0 0 1 0 0 0 0 0 0 0 0 0;
   0 0 0 0 0 0 1 0 0 0 0 0;
   0 0 0 0 0 0 0 1 0 0 0 0;
   0 0 0 0 0 0 0 0 1 0 0 0]

D=[0 0 0 0;
   0 0 0 0;
   0 0 0 0;
   0 0 0 0]

% LQR weight matrices
%q = [10 20 80 5 5 15 10 20 100 8 5 1]
q = [500 200 1.71 500 200 3 10 10 10 0.25 10 1]
Q = diag(q)
%r = [1 0.1 0.1 0.1]
r = [1 0.001 0.001 0.001]
R = diag(r)

N=0;

[K,S,e] = lqr(A,B,Q,R,N) % calculating the optimal gain matrix[K]

% Feedback conversion marices
H=[-kf -kf -kf -kf;
   0 -kf*l 0 kf*l;
   kf*l 0 -kf*l 0;
   km -km km -km]

L=1/det(H)

J =[0 0 kf*l km;
    0 -kf*l 0 -km;
    0 0 -kf*l km;
    0 kf*l 0 -km]

G=L*J

%set(gca,'FontSize',18);

