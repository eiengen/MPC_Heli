clc
clear all


Helicopter_para
%choose the operating point
%states
theta_op = -10*pi/180;
psi_op = pi/2;
w_theta_op = 0;
w_psi_op = 0;
%calculate the operating point for control inputs
Vmp_op = (Kyy*m_heli*g*cos(theta_op)*l_cm)/(Kyy*Kpp-Kyp*Kpy);
Vmy_op = Kyp*Vmp_op/Kyy;

% continuous time system matrices
A = [0,0,1,0;...
      0,0,0,1;...
      m_heli*g*l_cm*sin(theta_op)/(J_eq_p+m_heli*l_cm^2),0,-Bp/(J_eq_p+m_heli*l_cm^2),0;...
      (-Kyy*Vmy_op+Kyp*Vmp_op)*(2*m_heli*l_cm^2*sin(theta_op)*cos(theta_op))/(J_eq_y+m_heli*l_cm^2*(cos(theta_op))^2)^2,0,0, -By/(J_eq_y+m_heli*(l_cm*cos(theta_op))^2)];
 
 B = [0,0;...
       0,0;...
       Kpp/(J_eq_p+m_heli*l_cm^2),-Kpy/(J_eq_p+m_heli*l_cm^2);...
       Kyp/(J_eq_y+m_heli*(l_cm*cos(theta_op))^2), -Kyy/(J_eq_y+m_heli*(l_cm*cos(theta_op))^2)];
   
 C = [1,0,0,0;0,1,0,0];
 D = [0 0; 0 0];
 
% Calculate the steady state Kalman filter gain offline
%for measurment noise transmission matrix
G = eye(4); %this is difficult to know in reality. So assume it is identity matrix (i.e. 1)
H = zeros(2,4);
%weighting matrices for process noise and measurement noise
%plant model (knowledge about the process)
% dx = Ax + Bu + Gw
% y_plant = Cx + v
% w and v are process and measurment noises respectively
%plant = ss(A,[B,G],C,[D H]); %D = 0 since it is not present
plant = ss(A,[B,G],C,0); %D = 0 since it is not present
%%
Q = diag([5e1,7e1,8e1,6e1]); %process
R = diag([0.1,0.1]); %measurement noise
%note: If you trust your sensors, you can put relatively less R. If you do
%not trust your model, you can put relatively more Q.

%calculate the kalman gain L
[kalmf,L,P,M] = kalman(plant,Q,R);
 