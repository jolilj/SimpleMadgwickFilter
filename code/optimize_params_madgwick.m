function [theta_opt, t, x_opt] = optimize_params_madgwick(data, t)
%Calculates optimal param for the madgwick filter for the 
% last samples in a specific dataset using fminsearch
target = pi/2;
target = target*ones(1,size(t,1));
error_domain = 500;

x0 = 0.01; %Initial guess
x_opt = fminsearch(@opt,x0); %Run optimization
q_opt = madgwick_filter(data, t, x_opt); %Return filter results with optimal params
theta_opt = Quaternion2Theta(q_opt)'; %Convert to degrees

    function error = opt(x)
        %Returns rms error between target and estimate
        [q_est] = madgwick_filter(data,t,x);
        theta = Quaternion2Theta(q_est);
        error = sqrt(sum((theta(end-error_domain:end)' - target(end-error_domain:end)).^2))/error_domain;
    end
end