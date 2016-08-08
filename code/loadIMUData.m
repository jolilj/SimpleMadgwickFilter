function [ data, bias, variance, t] = loadIMUData(calib_data_file, data_file, dir)
%LOADIMUDATA Loads the imu data, calculates mean and corrects measurements
%   Returns corrected data, mean and variance
data = load(data_file);
t = data(:,10);
data = data(:,1:9);


calib_data = load(calib_data_file);
calib_data = calib_data(500:end,1:9);
%Calculate mean and variance
bias = mean(calib_data);
if (dir == 90) %Lying down
    bias(5)= bias(5) - 1;
end
if (dir == 0)%Standing up
   bias(6) = bias(6) - 1; 
end

variance = var(calib_data);

%Correct measurements
biasmat = repmat(bias, size(data,1),1);
data = data - biasmat;

end

