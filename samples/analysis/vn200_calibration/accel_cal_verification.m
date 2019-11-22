    % EE 440 Modern Nav
% Description:
%    Conduct a post cal test to verify cal procedure
% Author: S. Bruder

clear;                  % Clear all variables from the workspace
close all;              % Close all windows
clc;                    % "Clean" the command window

%% David Stockhouse

skewsym = @(v) [0    -v(3) v(2);
                v(3)   0  -v(1);
               -v(2)  v(1)  0  ];

rodriguez = @(theta, K) eye(3) + sin(theta) * K + (1 - cos(theta)) * K^2;

rotx = @(t) [1   0       0;
             0 cos(t) -sin(t);
             0 sin(t)  cos(t)];

roty = @(t) [ cos(t) 0 sin(t);
                0    1   0;
             -sin(t) 0 cos(t)];

rotz = @(t) [cos(t) -sin(t) 0;
             sin(t)  cos(t) 0;
               0       0    1];


g = 9.79193;            % Local value of gravity (m/s^2)

% Load the accel data file
[cal_file, cal_folder] = uigetfile('*cal*.mat', 'Select the calibration file to use for this accel cal verification:');
load([cal_folder, cal_file]);   % Load the cal parameters
Ma
b_aFB

%% Perform an accel Verification Test (stationary IMU)
% Load the accel data file to be used for verification
[verify_file, verf_folder] = uigetfile('*accel.mat', 'Select the x/y/z-up/down test file to use for accel cal verification:');
% Sample frequency (Hz) - Should be loaded from the verify data file
dT = 1/Fs;              % ?? Hz update rate - This was set in the data collection script
load([verf_folder, verify_file]);      % Load the test file (e.g., 'z_up_accel.mat')
fprintf('Using file: %s with Fs = %d Hz\n\n',  verify_file, Fs)

% If you use something other than a z-up accel data file you MUST change g_b
g_b = [-g;0;0];          % Gravity vector in the body frame (Y-down)

%% Compute the Calibrated accel, velocity, and position
cal_pos     = zeros(3, length(accel));      % Initialize the calibrated pos/vel/acc vectors
cal_vel     = zeros(3, length(accel));
cal_acc     = zeros(3, length(accel));
cal_dist    = zeros(1, length(accel));

% Compute the Calibrated position and velocity from accel measurements
for k = 1:length(accel)         
    f_uncal = accel(k,:)';                      % Uncalibrated accel measurements
    f_cal = (eye(3) + Ma)^-1 * (f_uncal - b_aFB); % Calibrated accel measurements
    cal_acc(:,k) = f_cal + g_b;                 % Remove accel due to gravity
    if k > 1
        a = cal_acc(:,k-1);       % Const accel during t_k-1 to t_k 
        cal_vel(:,k) = cal_vel(:,k-1) + dT * cal_acc(:,k-1);
        cal_pos(:,k) = cal_pos(:,k-1) + dT * cal_vel(:,k-1) + dT^2/2 * cal_acc(:,k-1);
        cal_dist(k)  = norm(cal_pos(:,k));
    end
end

%**************************************************************************
%% Compute the UnCalibrated accel, velocity, and position
%**************************************************************************
uncal_pos   = zeros(3, length(accel));      % Initialize the uncalibrated pos/vel/acc vectors
uncal_vel   = zeros(3, length(accel));
uncal_acc   = zeros(3, length(accel));
uncal_dist  = zeros(1, length(accel));

% Compute the UnCalibrated position and velocity from accel measurements
for k = 1:length(accel)    
    f_uncal = accel(k,:)';          % Uncalibrated accel measurements
    uncal_acc(:,k) = f_uncal + g_b; % Remove accel due to gravity
    if k > 1
        a = uncal_acc(:,k-1);       % Const accel during t_k-1 to t_k        
        uncal_vel(:,k) = uncal_vel(:,k-1) + dT * uncal_acc(:,k-1);
        uncal_pos(:,k) = uncal_pos(:,k-1) + dT * uncal_vel(:,k-1) + dT^2/2 * uncal_acc(:,k-1);
        uncal_dist(k)  = norm(uncal_pos(:,k));
    end
end

%% Plot the Calibrated / UnCalibrated accel, velocity, and position
t = 0:dT:(length(accel)-1)*dT;      % Time vector (sec)
figure('Units', 'normalized', 'Position', [0.01 0.05 0.5 0.4])
% Plot the Calibrated accel, velocity, and position
subplot(3,2,1)
plot(t, cal_acc(1,:), 'r', t, cal_acc(2,:), 'g', t, cal_acc(3,:),'b')
title('Calibrated')
%xlabel('Time (sec)')
ylabel('Accel (m/s^2)')
legend('a_x', 'a_y', 'a_z')
grid

subplot(3,2,3)
plot(t, cal_vel(1,:), 'r', t, cal_vel(2,:), 'g', t, cal_vel(3,:),'b')
%title('Calibrated Velocity')
%xlabel('Time (sec)')
ylabel('Vel (m/s)')
legend('v_x', 'v_y', 'v_z')
grid

subplot(3,2,5)
plot(t, cal_pos(1,:), 'r', t, cal_pos(2,:), 'g', t, cal_pos(3,:),'b')
%title('Calibrated Position')
xlabel('Time (sec)')
ylabel('Pos (m)')
legend('p_x', 'p_y', 'p_z')
grid

% Plot the UnCalibrated accel, velocity, and position
subplot(3,2,2)
plot(t, uncal_acc(1,:), 'r', t, uncal_acc(2,:), 'g', t, uncal_acc(3,:),'b')
title('UnCalibrated')
%xlabel('Time (sec)')
ylabel('Accel (m/s^2)')
legend('a_x', 'a_y', 'a_z')
grid

subplot(3,2,4)
plot(t, uncal_vel(1,:), 'r', t, uncal_vel(2,:), 'g', t, uncal_vel(3,:),'b')
%title('UnCalibrated Velocity')
%xlabel('Time (sec)')
ylabel('Vel (m/s)')
legend('v_x', 'v_y', 'v_z')
grid

subplot(3,2,6)
plot(t, uncal_pos(1,:), 'r', t, uncal_pos(2,:), 'g', t, uncal_pos(3,:),'b')
%title('UnCalibrated Position')
xlabel('Time (sec)')
ylabel('Pos (m)')
legend('p_x', 'p_y', 'p_z')
grid

%% Compare the calibrated and uncalibrated cases

% 3D Stem plot of Calibrated position
figure('Units', 'normalized', 'Position', [0.5 0.05 0.5 0.4])
subplot(1,2,1)
stem3(cal_pos(1,1:5:end), cal_pos(2,1:5:end), cal_pos(3,1:5:end))
title('Plot of the CALIBRATED Position')
xlabel('x_{pos} (m)')
ylabel('y_{pos} (m)')
zlabel('z_{pos} (m)')

fprintf('Calibrated Case:\n   End Pos = [%4.1f, %4.1f, %4.1f] (m)\n', cal_pos(1, end), cal_pos(2, end), cal_pos(3, end))
fprintf('  Total distance Traveled = %4.1f\n\n', norm(cal_pos(:, end)))

subplot(1,2,2)
stem3(uncal_pos(1,1:5:end), uncal_pos(2,1:5:end), uncal_pos(3,1:5:end))
title('Plot of the UN-CALIBRATED Position')
xlabel('x_{pos} (m)')
ylabel('y_{pos} (m)')
zlabel('z_{pos} (m)')

fprintf('UnCalibrated Case:\n   End Pos = [%4.1f, %4.1f, %4.1f] (m)\n', uncal_pos(1, end), uncal_pos(2, end), uncal_pos(3, end))
fprintf('  Total distance Traveled = %4.1f\n\n', norm(uncal_pos(:, end)))

% Compare the calibrated and uncalibrated distances (norm of pos)
figure('Units', 'normalized', 'Position', [0.25 0.5 0.3 0.4])
plot(t, uncal_dist, 'r', t, cal_dist, 'b', 'LineWidth', 2)
title('Dist Errors for the Cal vs Uncal Accels')
ylabel('Dist (m)')
xlabel('Time (sec)')
legend('Uncalibrated', 'Calibrated')
grid