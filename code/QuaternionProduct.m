function [ Q ] = QuaternionProduct(Q1, Q2)
%QUATERNIONPRODUCT
% This function returns the quaternion product of two quaternions

    Q(:,1) = Q1(:,1).*Q2(:,1) - Q1(:,2).*Q2(:,2) - Q1(:,3).*Q2(:,3) - Q1(:,4).*Q2(:,4);
    Q(:,2) = Q1(:,1).*Q2(:,2) + Q1(:,2).*Q2(:,1) + Q1(:,3).*Q2(:,4) - Q1(:,4).*Q2(:,3);
    Q(:,3) = Q1(:,1).*Q2(:,3) - Q1(:,2).*Q2(:,4) + Q1(:,3).*Q2(:,1) + Q1(:,4).*Q2(:,2);
    Q(:,4) = Q1(:,1).*Q2(:,4) + Q1(:,2).*Q2(:,3) - Q1(:,3).*Q2(:,2) + Q1(:,4).*Q2(:,1);
    

end



