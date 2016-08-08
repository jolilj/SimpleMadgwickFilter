function [mu_opt, Sigma, t, x_opt] = optimize_params_kalman(data,t)
% Calculates the optimal params for the kalman for the last samples of a specific 
% dataset filter using fminsearch

target = pi/2; % Target is based on the data, here it's pi/2
target = target*ones(1,size(t,1));
error_domain = 500;
x0 = [1, 1, 1]; %Initial guess

x_opt = fminsearch(@opt,x0); %Run the optimization

[mu_opt ,Sigma] = kalman_filter(data, t, x_opt); %Return filter result with opt params

    function error = opt(x)
        %Returns the rms error between the target and the kalman estimate
        [mu, ~] = kalman_filter(data,t,x);
        error = sqrt(sum((mu(1,end-error_domain:end) - target(end-error_domain:end)).^2))/error_domain;
    end
end