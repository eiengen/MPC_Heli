%% This file loads the helicopter system parameters

l_cm   = 0.015;   % [m] - Distance between the pivot point and the center of mass of the helicopter.
m_heli = 0.479;   % [kg] - Total moving mass of the helicopter
J_eq_p = 0.0172;  % [kg-m^2] - Moment of inertia about pitch axis
J_eq_y = 0.0210;  % [kg-m^2] - Moment of inertia about yaw axis
g      = 9.81;    % [m/s^2] - Acceleration due to gravity on planet earth

Kpp    = 0.0556;  % [Nm/V] - Torque constant on pitch axis from pitch motor
Kyy    = 0.21084; % [Nm/V] - Torque constant on yaw axis from yaw motor
Kpy    = 0.005;   % [Nm/V] - Torque constant on pitch axis from yaw motor
Kyp    = 0.15;    % [Nm/V] - Torque constant on yaw axis from pitch motor
Bp     = 0.01;    % [N/V]  - Damping friction factor about pitch axis
By     = 0.08;    % [N/V]  - Damping friction factor about yaw axis


