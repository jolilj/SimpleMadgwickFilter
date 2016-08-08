function [mu, Sigma] = kalman_filter(data, t, x)
    t = t-t(1);
    r_1 = x(1);
    r_2 = x(2);
    q = x(3);
    %Get adequate data for Kalman filter (omega_x, a_y, a_z)
    z = [data(:,1)';data(:,5)';data(:,6)'];
    %Process noise model
    R = eye(2);
    R(1,1) = r_1;
    R(2,2) = r_2;

    %Measurement noise model, estimated from measurements
    Q = eye(3)*q;

    %Initial guess
    mu_0 = [0;0];
    Sigma_0 = eye(2);
    Sigma_0(1,1) = 1;
    Sigma_0(2,2) = 1;
    
    %Run filter
    [mu, Sigma] = kalman_alg(mu_0, Sigma_0, z, R, Q, t);
end