function [delta_x_hat, P, Kf] = linear_state_observer(delta_x,P, delta_u,delta_y_m,Ac,Bc,Cc,L,R, Q)
    
    % Compute apriori state covariance
    P = Ac*P*Ac' + Q;
    % Kalman gain
    Kf = (P*Cc')/(Cc*P*Cc'+R);
    %apriori state
    delta_x_hat = Ac*delta_x + Bc*delta_u;
    %estimated output    
    y_hat = Cc*delta_x;
    % aposteriori state
    delta_x_hat = delta_x_hat + Kf*(delta_y_m - y_hat);
    % Update state covariance
    P = (eye(size(Kf*Cc)) - Kf*Cc)*P*(eye(size(Kf*Cc))-Kf*Cc)' + Kf*R*Kf';
