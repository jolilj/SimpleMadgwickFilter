%% Main Kalman alg
function [mu, Sigma] = kalman_alg( mu_0, Sigma_0, z, R, Q, t )
%KALMAN Kalman algorithm
%   Takes initial guess mu_0, intial variance Sigma_0, measurements z,
%   covariance matrices of process noise and measurement noise R and Q
%   respectively and a time vector. Runs the Kalman algorithm and returns
%   the mean of the state vector, mu.
    
    % Initalize
    N = size(z,2);
    mu = zeros(2,N);
    Sigma = zeros(2,2,N);

    %Init with initial conditions
    mu(1,1) = mu_0(1);
    mu(2,1) = mu_0(2);
    Sigma(:,:,1) = Sigma_0;
    z = [zeros(3,1), z];
  
    %Kalman alg
    for i=2:N
        deltaT = t(i) - t(i-1);
        A = [1, deltaT;0, 1];

        mu_bar = A*mu(:,i-1);
        H = [0, 1;cos(mu_bar(1)), 0;-sin(mu_bar(1)), 0];
        Sigma_bar = A*Sigma(:,:,i-1)*A' + R;

        K=Sigma_bar*H'/(H*Sigma_bar*H'+Q);
        mu(:,i) = mu_bar + K*nu(z(:,i), mu_bar); 
        Sigma(:,:,i) = (eye(2)-K*H)*Sigma_bar;
    end
end
%% Innovation
function nu = nu(z, x)
    %Calculates the innovation of the measurement z and mean x
    nu = z - [x(2) sin(x(1)) cos(x(1))]';
end
