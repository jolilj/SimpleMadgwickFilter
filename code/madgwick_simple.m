function [ q_est ] = madgwick_simple( q_0, omega, a, beta, t)
%MADGWICK filter algorithm without the magnetometer measurements
% q_0 - initial quaternion guess
% omega - gyroscope measurements
% a - accelerometer measurements
% beta - filter parameter
% t - time vector

   %Init
   %disp(omega)
   N = size(omega,1);
   q_est = zeros(N,4);
   
   %Initial condition
   q_est(1,:) = q_0;
   omega = [zeros(3,1), omega']';
   a = [zeros(3,1), a']';
   for i=2:N
      deltaT = t(i) - t(i-1);
      q_dot = 0.5*QuaternionProduct(q_est(i-1,:),[0,omega(i,:)]);
      q_omega = q_est(i-1,:) + q_dot*deltaT;
      q_grad = -gradf(q_est(i-1,:),a(i,:));
      q_temp = beta*deltaT*q_grad + q_omega ;
      q_est(i,:) = q_temp/norm(q_temp);
   end

end

%% 
function q_grad = gradf(q,a)
% Calculates the accelerometer estimate of the quaternion representation of
% the orientation
    a = a/norm(a);
    J_g = [-2*q(3), 2*q(4), -2*q(1), 2*q(2);...
            2*q(2), 2*q(1),  2*q(4), 2*q(3);...
            0,     -4*q(2), -4*q(3), 0];
        
    f_g = [2*(q(2)*q(4)-q(1)*q(3)) - a(1);...
           2*(q(1)*q(2) + q(3)*q(4))-a(2);...
           2*(0.5-q(2)^2-q(3)^2)-a(3)];
       
    q_grad = J_g'*f_g;
    q_grad = q_grad'/norm(q_grad);
end