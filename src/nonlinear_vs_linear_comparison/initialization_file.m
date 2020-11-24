% This is the initialization file. This has to be run first before running
% the simulink file. It will create the Ac, Bc, Cc matrices
% based on the given choice of operating point.
% Inside the linear model block in simulink, the matrices Ac, Bc and Cc is passed to the matlab
% funciton block as input arguments using a constant block. In the constant
% block you can simply type Ac and it will be loaded from the workspace.

clc
clear all


%Helicopter_para
l_cm = 1.5/100; % distance between CG and pivot point

% % found from experiments and parameter estimation
 Kpp = 0.0556;
 Kyy = 0.21084;
% 
 Kpy = 0.005;
 Kyp = 0.15;
 Bp = 0.01;   
 By = 0.08; 

m_heli = 479/1000; %mass of helicopter (total moving mass)

J_eq_p = 0.0172;

J_eq_y = 0.0210;%0.0173;%

g = 9.81;

%operating point
%states
theta_op = -10*pi/180;
psi_op = pi/2;
w_theta_op = 0;
w_psi_op = 0;
%soperating point inputs inputs
Vmp_op = (Kyy*m_heli*g*cos(theta_op)*l_cm)/(Kyy*Kpp-Kyp*Kpy);
Vmy_op = Kyp*Vmp_op/Kyy;

Ac = [0,0,1,0;...
      0,0,0,1;...
      m_heli*g*l_cm*sin(theta_op)/(J_eq_p+m_heli*l_cm^2),0,-Bp/(J_eq_p+m_heli*l_cm^2),0;...
      (-Kyy*Vmy_op+Kyp*Vmp_op)*(2*m_heli*l_cm^2*sin(theta_op)*cos(theta_op))/(J_eq_y+m_heli*l_cm^2*(cos(theta_op))^2)^2,0,0, -By/(J_eq_y+m_heli*(l_cm*cos(theta_op))^2)];
 
 Bc = [0,0;...
       0,0;...
       Kpp/(J_eq_p+m_heli*l_cm^2),-Kpy/(J_eq_p+m_heli*l_cm^2);...
       Kyp/(J_eq_y+m_heli*(l_cm*cos(theta_op))^2), -Kyy/(J_eq_y+m_heli*(l_cm*cos(theta_op))^2)];
   
 Cc = [1,0,0,0;0,1,0,0];
 
 

