function delta_x_hat = linear_state_observer(delta_x, delta_u,delta_y_m,Ac,Bc,Cc,L)
    %apriori state
    delta_x_hat = Ac*delta_x + Bc*delta_u;
    %estimated output    
    y_hat = Cc*delta_x;
    % aposteriori state
    delta_x_hat = delta_x_hat + L*(delta_y_m - y_hat);