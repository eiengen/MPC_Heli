clc
clear all


Helicopter_para
%select the operating points
%states
theta_op = -10*pi/180;
psi_op = pi/2;
w_theta_op = 0;
w_psi_op = 0;
%Calculate the operating point for control inputs
Vmp_op = (Kyy*m_heli*g*cos(theta_op)*l_cm)/(Kyy*Kpp-Kyp*Kpy);
Vmy_op = Kyp*Vmp_op/Kyy;

%initialization of the UKF
P_plus_ini = zeros(4,4).*0.3;
x_plus_ini = [theta_op;psi_op;w_theta_op;w_psi_op];

%you can play around with the weights and see how it affects the estimation 
Q = diag([1e-2,1e5,1e-2,1e5]);
 R = diag([1e-2,1e-3]);
