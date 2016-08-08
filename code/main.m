%% Optimization, optimize the params for the filters given a specific dataset
close all; clc; clear;
[data, ~, ~, t] = loadIMUData('data/ImuDataCalib90Deg.txt','data/ImuData0-90-1.txt', 90);
t = t-t(1);
[theta_opt, ~, x_opt_mad] = optimize_params_madgwick(data, t);
[mu_opt, Sigma, ~, x_opt_kal] = optimize_params_kalman(data, t);

%% Plot example
close all;
[mu1, Sigma1] = kalman_filter(data,t,[0.01, 1, 0.01]);
[mu2, Sigma2] = kalman_filter(data,t,[0.01, 1, 2]);
[mu3, Sigma3] = kalman_filter(data,t,[0.01, 0.01, 100]);

q1 = madgwick_filter(data,t,2);
q2 = madgwick_filter(data,t,1);
q3 = madgwick_filter(data,t,0.1);

target = pi/2*ones(1,size(t,1));

figure();
hold on;
plot(t,180/pi*mu1(1,:));
plot(t,180/pi*(mu2(1,:))+5);
plot(t,180/pi*(mu3(1,:))+10);
plot(t,180/pi*(mu_opt(1,:))+15);
plot(t,180/pi*target,'k');
plot(t,180/pi*(target)+5,'k');
plot(t,180/pi*(target)+10,'k');
plot(t,180/pi*(target)+15,'k');
axis([0 23 -9 110]);
title('Extended Kalman Filter');
legend('Parameters #1',...
       'Parameters #2',...
       'Parameters #3',...
       'Optimal Parameters','Location','east');
xlabel('t[s]');
ylabel('\theta [°]');

figure();
hold on;
plot(t,180/pi*Quaternion2Theta(q1));
plot(t,180/pi*(Quaternion2Theta(q2))+5);
plot(t,180/pi*(Quaternion2Theta(q3))+10);
plot(t,180/pi*(theta_opt)+15);
plot(t,180/pi*target,'k');
plot(t,180/pi*(target)+5,'k');
plot(t,180/pi*(target)+10,'k');
plot(t,180/pi*(target)+15,'k');
axis([0 23 -9 110]);
title('Madgwick');
legend('Parameters #1',...
       'Parameters #2',...
       'Parameters #3',...
       'Optimal Parameters','Location','east');
xlabel('t[s]');
ylabel('\theta [°]');

%% Plot example zoomed in
close all;
figure();
hold on;
plot(t,180/pi*mu1(1,:));
plot(t,180/pi*(mu2(1,:))+5);
plot(t,180/pi*(mu3(1,:))+10);
plot(t,180/pi*(mu_opt(1,:))+15);
plot(t,180/pi*target,'k');
plot(t,180/pi*(target)+5,'k');
plot(t,180/pi*(target)+10,'k');
plot(t,180/pi*(target)+15,'k');
axis([12 23 80 110]);
title('Extended Kalman Filter');
legend('Parameters #1',...
       'Parameters #2',...
       'Parameters #3',...
       'Optimal Parameters','Location','southeast');
xlabel('t[s]');
ylabel('\theta [°]');

figure();
hold on;
plot(t,180/pi*Quaternion2Theta(q1));
plot(t,180/pi*(Quaternion2Theta(q2))+5);
plot(t,180/pi*(Quaternion2Theta(q3))+10);
plot(t,180/pi*(theta_opt)+15);
plot(t,180/pi*target,'k');
plot(t,180/pi*(target)+5,'k');
plot(t,180/pi*(target)+10,'k');
plot(t,180/pi*(target)+15,'k');
axis([12 23 80 110]);
title('Madgwick');
legend('Parameters #1',...
       'Parameters #2',...
       'Parameters #3',...
       'Optimal Parameters','Location','southeast');
xlabel('t[s]');
ylabel('\theta [°]');

%% Load all datasets for filter performance comparison
data = [{},{},{},{},{},{},{},{}];
[data(1).data, ~, ~, data(1).t] = loadIMUData('data/ImuDataCalib90Deg.txt','data/ImuData0-90-1.txt', 90);
[data(2).data, ~, ~, data(2).t] = loadIMUData('data/ImuDataCalib90Deg.txt','data/ImuData0-90-2.txt', 90);
[data(3).data, ~, ~, data(3).t] = loadIMUData('data/ImuDataCalib90Deg.txt','data/ImuData0-90-3.txt', 90);
[data(4).data, ~, ~, data(4).t] = loadIMUData('data/ImuDataCalib90Deg.txt','data/ImuData90-90-1.txt', 90);
[data(5).data, ~, ~, data(5).t] = loadIMUData('data/ImuDataCalib90Deg.txt','data/ImuData90-90-2.txt', 90);
[data(6).data, ~, ~, data(6).t] = loadIMUData('data/ImuDataCalib90Deg.txt','data/ImuData90-90-3.txt', 90);
%% Run filters, register error and time elapsed for each dataset and calculate the mean
target = pi/2;
error_kal = zeros(1,6);
error_mad = zeros(1,6);
error_domain = 200;
t_ekf = zeros(1,6);
t_mad = zeros(1,6);
for i=1:6
   ref = target*ones(1,size(data(i).t,1));
   tic;
   [mu,~] = kalman_filter(data(i).data, data(i).t,x_opt_kal);
   t_ekf(i) = toc;
   tic
   q = madgwick_filter(data(i).data, data(i).t, 0.2);
   theta = Quaternion2Theta(q);
   theta = theta';
   t_mad(i) = toc;
   
   error_kal(i) = rms(ref(end-error_domain:end)-mu(1,end-error_domain:end));
   error_mad(i) = rms(ref(end-error_domain:end)-theta(end-error_domain:end));
end

e_kal = mean(error_kal) %Mean error of the kalman filter
t_kal = mean(t_ekf) %Mean time elapsed for the kalman filter

e_mad = mean(error_mad) %Mean error of the madgwick filter
t_m = mean(t_mad) %Mean time elapsed for the madgwick filter
