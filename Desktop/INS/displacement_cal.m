%% Housekeeping

addpath('ximu_matlab_library');  % include x-IMU MATLAB library
addpath('Quaternions');   % include quaternion library
addpath('C:\Users\CG JAYASHREE\Desktop\Madgwick AHRS\Oscillatory-Motion-Tracking-With-x-IMU\SixDofAnimation')
close all;                       % close all figures
clear;                           % clear all variables
clc;                             % clear the command terminal

%% Import Data

% Read the CSV file
filename = "C:\Users\CG JAYASHREE\Desktop\final\Gait Tracking With x-IMU\sensor_data.csv";    % Name of your CSV file
data = readtable(filename);

% Extract accelerometer and gyroscope data
gyr = table2array(data(:, 5:7)); % Gyroscope columns (X, Y, Z)
acc = table2array(data(:, 2:4)); % Accelerometer columns (X, Y, Z)

% Define sample period (as per your data logging settings)
samplePeriod = 1/200;

%% Plot

% Plot Gyroscope
figure('NumberTitle', 'off', 'Name', 'Gyroscope');
hold on;
plot(gyr(:,1), 'r');
plot(gyr(:,2), 'g');
plot(gyr(:,3), 'b');
xlabel('Sample');
ylabel('dps');
title('Gyroscope');
legend('X', 'Y', 'Z');

% Plot Accelerometer
figure('NumberTitle', 'off', 'Name', 'Accelerometer');
hold on;
plot(acc(:,1), 'r');
plot(acc(:,2), 'g');
plot(acc(:,3), 'b');
xlabel('Sample');
ylabel('g');
title('Accelerometer');
legend('X', 'Y', 'Z');

%% Process Data through AHRS Algorithm (Calculate Orientation)
% See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

R = zeros(3,3,length(gyr));     % Rotation matrix describing sensor relative to Earth

ahrs = AHRS('SamplePeriod', samplePeriod, 'Kp', 1);

for i = 1:length(gyr)
    ahrs.UpdateIMU(gyr(i,:) * (pi/180), acc(i,:)); % Gyroscope units must be radians
    R(:,:,i) = quatern2rotMat(ahrs.Quaternion)'; % Transpose because ahrs provides Earth relative to sensor
end

%% Calculate 'Tilt-Compensated' Accelerometer

tcAcc = zeros(size(acc));  % Accelerometer in Earth frame

for i = 1:length(acc)
    tcAcc(i,:) = R(:,:,i) * acc(i,:)';
end

% Plot Tilt-Compensated Accelerometer
figure('NumberTitle', 'off', 'Name', '''Tilt-Compensated'' Accelerometer');
hold on;
plot(tcAcc(:,1), 'r');
plot(tcAcc(:,2), 'g');
plot(tcAcc(:,3), 'b');
xlabel('Sample');
ylabel('g');
title('''Tilt-Compensated'' Accelerometer');
legend('X', 'Y', 'Z');

%% Calculate Linear Acceleration in Earth Frame (Subtracting Gravity)

linAcc = tcAcc - [zeros(length(tcAcc), 1), zeros(length(tcAcc), 1), ones(length(tcAcc), 1)];
linAcc = linAcc * 9.81;     % Convert from 'g' to m/s/s

% Plot Linear Acceleration
figure('NumberTitle', 'off', 'Name', 'Linear Acceleration');
hold on;
plot(linAcc(:,1), 'r');
plot(linAcc(:,2), 'g');
plot(linAcc(:,3), 'b');
xlabel('Sample');
ylabel('m/s^2');
title('Linear Acceleration');
legend('X', 'Y', 'Z');

%% Calculate Linear Velocity (Integrate Acceleration)

linVel = zeros(size(linAcc));

for i = 2:length(linAcc)
    linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * samplePeriod;
end

% Plot Linear Velocity
figure('NumberTitle', 'off', 'Name', 'Linear Velocity');
hold on;
plot(linVel(:,1), 'r');
plot(linVel(:,2), 'g');
plot(linVel(:,3), 'b');
xlabel('Sample');
ylabel('m/s');
title('Linear Velocity');
legend('X', 'Y', 'Z');

%% High-Pass Filter Linear Velocity to Remove Drift

order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linVelHP = filtfilt(b, a, linVel);

% Plot High-Pass Filtered Linear Velocity
figure('NumberTitle', 'off', 'Name', 'High-Pass Filtered Linear Velocity');
hold on;
plot(linVelHP(:,1), 'r');
plot(linVelHP(:,2), 'g');
plot(linVelHP(:,3), 'b');
xlabel('Sample');
ylabel('m/s');
title('High-Pass Filtered Linear Velocity');
legend('X', 'Y', 'Z');

%% Calculate Linear Position (Integrate Velocity)

linPos = zeros(size(linVelHP));

for i = 2:length(linVelHP)
    linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) * samplePeriod;
end

% Plot Linear Position
figure('NumberTitle', 'off', 'Name', 'Linear Position');
hold on;
plot(linPos(:,1), 'r');
plot(linPos(:,2), 'g');
plot(linPos(:,3), 'b');
xlabel('Sample');
ylabel('m');
title('Linear Position');
legend('X', 'Y', 'Z');

%% High-Pass Filter Linear Position to Remove Drift

order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linPosHP = filtfilt(b, a, linPos);

% Plot High-Pass Filtered Linear Position
figure('NumberTitle', 'off', 'Name', 'High-Pass Filtered Linear Position');
hold on;
plot(linPosHP(:,1), 'r');
plot(linPosHP(:,2), 'g');
plot(linPosHP(:,3), 'b');
xlabel('Sample');
ylabel('m');
title('High-Pass Filtered Linear Position');
legend('X', 'Y', 'Z');

%% Play Animation

SamplePlotFreq = 8;

SixDofAnimation(linPosHP, R, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));            

%% End of Script
