function [ phi ] = Quaternion2Theta( Q )
%QUATERNION2THETA Convert quaternion to pitch
%   Converting the quaternion to return phi angle around x-axis
    phi = -atan2(2*(Q(:,3).*Q(:,4)-2*Q(:,1).*Q(:,2)),2*Q(:,1).^2+2*Q(:,4).^2-1);
end
