%% Linearize the helicopter model
% State space liniarization

% Define model parameters
system_parameters
% Operating points for the states
th_op    = -10*pi/180; yw_op    = pi/2;
w_th_op  = 0;          w_yw_op  = 0;
% Operating points for the inputs
Vmp_op = (Kyy*m_heli*g*cos(th_op)*l_cm) / (Kyy*Kpp - Kyp*Kpy);
Vmy_op = (Kyp*Vmp_op) / (Kyy);

% Compute jacobians for Taylor series linearization
% Partial derivatives of functions f1 to f4 for the state equations.
% (Simplyfied by removing products of w_th_op and w_yw_op which becomes
% zero)
d_f1_dth = 0; d_f1_dyw = 0; d_f1_dw_th = 1; d_f1_dw_yw = 0;
d_f2_dth = 0; d_f2_dyw = 0; d_f2_dw_th = 0; d_f2_dw_yw = 1;
d_f3_dth = (m_heli * l_cm * g*sin(th_op)) / (J_eq_p + m_heli*l_cm^2); 
d_f3_dyw = 0; 
d_f3_dw_th = -Bp / (J_eq_p + m_heli*l_cm^2); 
d_f3_dw_yw = 0;
d_f4_dth =((Kyp*Vmp_op - Kyy*Vmy_op)*(2*m_heli*l_cm^2*sin(th_op)*cos(th_op))) / ...
          (J_eq_y + m_heli*l_cm^2*cos(th_op)^2)^2; 
d_f4_dyw = 0; 
d_f4_dw_th = 0; 
d_f4_dw_yw = - By / (J_eq_y + m_heli*l_cm^2*cos(th_op)^2);

% Partial derivatives of functions f1 to f4 for the input equations.
d_f1_Vmp = 0;
d_f1_Vmy = 0;
d_f2_Vmp = 0;
d_f2_Vmy = 0;
d_f3_Vmp = Kpp / (J_eq_p + m_heli*l_cm^2);
d_f3_Vmy = -Kpy /(J_eq_p + m_heli*l_cm^2);
d_f4_Vmp = Kyp / (J_eq_y + m_heli*l_cm^2*cos(th_op)^2);
d_f4_Vmy = -Kyy / (J_eq_y + m_heli*l_cm^2*cos(th_op)^2);

Ac = [d_f1_dth,d_f1_dyw,d_f1_dw_th,d_f1_dw_yw
      d_f2_dth,d_f2_dyw,d_f2_dw_th,d_f2_dw_yw
      d_f3_dth,d_f3_dyw,d_f3_dw_th,d_f3_dw_yw
      d_f4_dth,d_f4_dyw,d_f4_dw_th,d_f4_dw_yw];
  
Bc = [d_f1_Vmp,d_f1_Vmy
      d_f2_Vmp,d_f2_Vmy
      d_f3_Vmp,d_f3_Vmy
      d_f4_Vmp,d_f4_Vmy];

% Measurement matrix.
% Measured states, hence two outputs resulting in two rows in Cc
Cc = [1,0,0,0;0,1,0,0];
