function [ Q_conj ] = QuaternionConjugate( Q )
%QUATERNIONCONJUGATE
%   It returns the conjugate of the quaternion 

Q_conj = [Q(:,1) -Q(:,2) -Q(:,3) -Q(:,4)];

end


