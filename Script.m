%% Housekeeping
addpath('ximu_matlab_library'); % x-IMU MATLAB library
addpath("C:\Users\karth\Desktop\MPU9250\quaternion_library"); % Quaternion library
addpath("C:\Users\karth\Desktop\MPU9250\SixDOFanimation.m");

close all; % Close all figures
clear; % Clear all variables
clc; % Clear the command terminal

%% Import Data
filename = "C:\Users\karth\Desktop\MPU9250\sensor_data1.csv"; 
data = readtable(filename);

% Extract Accelerometer & Gyroscope Data
gyr = table2array(data(:, 5:7)) * (pi/180); % Convert deg/s to rad/s
acc = table2array(data(:, 2:4)); % Accelerometer (raw data)

% Sampling period based on data logging rate
samplePeriod = 1/200;

%% Plot Gyroscope Data
figure('NumberTitle', 'off', 'Name', 'Gyroscope');
hold on;
plot(gyr(:,1), 'r'); plot(gyr(:,2), 'g'); plot(gyr(:,3), 'b');
xlabel('Sample'); ylabel('rad/s'); title('Gyroscope');
legend('X', 'Y', 'Z');

%% Plot Accelerometer Data
figure('NumberTitle', 'off', 'Name', 'Accelerometer');
hold on;
plot(acc(:,1), 'r'); plot(acc(:,2), 'g'); plot(acc(:,3), 'b');
xlabel('Sample'); ylabel('g'); title('Accelerometer');
legend('X', 'Y', 'Z');

%% AHRS Algorithm (Mahony Filter for Orientation Estimation)
R = zeros(3,3,length(gyr)); % Rotation matrices
ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1, 'Ki', 0);

quaternion = zeros(length(gyr), 4); % Quaternion storage

for i = 1:length(gyr)
    ahrs.UpdateIMU(gyr(i,:), acc(i,:));
    quaternion(i, :) = ahrs.Quaternion; % Store Quaternion
    R(:,:,i) = quatern2rotMat(ahrs.Quaternion)'; % Rotation matrix
end

%% Calculate 'Tilt-Compensated' Accelerometer
tcAcc = zeros(size(acc));

for i = 1:length(acc)
    tcAcc(i,:) = R(:,:,i) * acc(i,:)';
end

% Plot Tilt-Compensated Acceleration
figure('NumberTitle', 'off', 'Name', '''Tilt-Compensated'' Accelerometer');
hold on;
plot(tcAcc(:,1), 'r'); plot(tcAcc(:,2), 'g'); plot(tcAcc(:,3), 'b');
xlabel('Sample'); ylabel('g'); title('Tilt-Compensated Accelerometer');
legend('X', 'Y', 'Z');

%% Linear Acceleration (Gravity Removed)
linAcc = zeros(size(tcAcc));

for i = 1:length(tcAcc)
    gravity = R(:,:,i) * [0; 0; 1]; % Gravity in sensor frame
    linAcc(i,:) = tcAcc(i,:) - gravity'; % Subtract gravity
end

linAcc = linAcc * 9.81; % Convert to m/s²

% Plot Linear Acceleration
figure('NumberTitle', 'off', 'Name', 'Linear Acceleration');
hold on;
plot(linAcc(:,1), 'r'); plot(linAcc(:,2), 'g'); plot(linAcc(:,3), 'b');
xlabel('Sample'); ylabel('m/s²'); title('Linear Acceleration');
legend('X', 'Y', 'Z');

%% Velocity Estimation (Integration of Acceleration)
linVel = zeros(size(linAcc));

for i = 2:length(linAcc)
    linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * samplePeriod;
end

% High-Pass Filter to Reduce Drift
order = 2; filtCutOff = 0.1; % Adjust cutoff as needed
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linVelHP = filtfilt(b, a, linVel);

% Plot High-Pass Filtered Velocity
figure('NumberTitle', 'off', 'Name', 'High-Pass Filtered Velocity');
hold on;
plot(linVelHP(:,1), 'r'); plot(linVelHP(:,2), 'g'); plot(linVelHP(:,3), 'b');
xlabel('Sample'); ylabel('m/s'); title('Filtered Linear Velocity');
legend('X', 'Y', 'Z');

%% Position Estimation (Double Integration of Acceleration)
linPos = zeros(size(linVelHP));

for i = 2:length(linVelHP)
    linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) * samplePeriod;
end

% High-Pass Filter to Remove Position Drift
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linPosHP = filtfilt(b, a, linPos);

% Plot High-Pass Filtered Position
figure('NumberTitle', 'off', 'Name', 'Filtered Position');
hold on;
plot(linPosHP(:,1), 'r'); plot(linPosHP(:,2), 'g'); plot(linPosHP(:,3), 'b');
xlabel('Sample'); ylabel('m'); title('Filtered Position');
legend('X', 'Y', 'Z');

%% 3D Animation of Motion (SixDOF)
SamplePlotFreq = 8;

SixDOFanimation(linPosHP, R, ...
    'SamplePlotFreq', SamplePlotFreq, 'Trail', 'All', ...
    'Position', [9 39 1280 720], ...
    'AxisLength', 0.1, 'ShowArrowHead', false, ...
    'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', ...
    'ShowLegend', false, 'Title', 'Filtered Motion', ...
    'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));

%% End of Script
