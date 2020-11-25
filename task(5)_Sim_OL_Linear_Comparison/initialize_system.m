clearvars
clc

% Define model parameters
heli_parameters
% Operating points for the states
th_op    = -10*pi/180; 
yw_op    = pi/2;
w_th_op  = 0;          
w_yw_op  = 0;
% Operating points for the inputs
Vmp_op = (Kyy*m_heli*g*cos(th_op)*l_cm) / (Kyy*Kpp - Kyp*Kpy);
Vmy_op = (Kyp*Vmp_op) / (Kyy);

% Compute linearized Ac, Bc and Cc
linearize_heli
