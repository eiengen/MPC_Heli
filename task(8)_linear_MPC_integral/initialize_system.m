
clearvars
clc

%% Load model parameters
heli_parameters

%% Configure operating points for the states
th_op    = -30*pi/180; 
yw_op    = pi/2;
w_th_op  = 0;          
w_yw_op  = 0;

%% Configure operating points for the inputs
Vmp_op = (Kyy*m_heli*g*cos(th_op)*l_cm) / (Kyy*Kpp - Kyp*Kpy);
Vmy_op = (Kyp*Vmp_op) / (Kyy);

%% Integral action parameter
Ki_pitch = 0.1;
Ki_yaw   = 0.1;

%% Compute linearized system matrices Ac, Bc and Cc
linearize_heli

%% Discrete time state space model to be used with linear MPC block
dt  = 0.1;            %sampling time
sys = ss(Ac,Bc,Cc,0); %there is no D matrix
ds  = c2d(sys,dt);
Ad  = ds.a; Bd = ds.b; Cd = ds.c; %discrete time system matrices

%% Calculate the steady state Kalman filter gain offline
%for measurment noise transmission matrix
G = eye(4); %this is difficult to know in reality. So assume it is identity matrix (i.e. 1)
H = zeros(2,4);
%weighting matrices for process noise and measurement noise
%plant model (knowledge about the process)
% dx = Ax + Bu + Gw
% y_plant = Cx + v
% w and v are process and measurment noises respectively
%plant = ss(A,[B,G],C,[D H]); %D = 0 since it is not present
plant = ss(Ac,[Bc,G],Cc,0); %D = 0 since it is not present
%
Q = diag([5e1,4e1,8e1,6e1]); %process
R = diag([0.1,0.1]); %measurement noise
%note: If you trust your sensors, you can put relatively less R. If you do
%not trust your model, you can put relatively more Q.

%calculate the initial kalman gain L
[kalmf,L,P,M] = kalman(plant,Q,R);
 