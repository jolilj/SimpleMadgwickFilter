function q_est = madgwick_filter(data, t, beta)
% Runs the filter for a specific data set
    
    % Retrieve data(gyro and accelerometer data)
    omega = data(:,1:3);
    a = data(:,4:6);
    a = a /norm(a);

    q_0 = [1 0 0 0]; % Initial guess
    
    % Run the filter
    q_est = madgwick_simple(q_0, omega, a, beta, t);
end