% EE 440 Modern Nav
% Description:
%   Script to calibrate the VN200 IMU
%   This script expects 6 data files correcponding to the 6-configurations
%   z_up.mat, z_down.mat, y_up.mat, y_down.mat, x_up.mat, x_down.mat
%   each containing accel(:,3) in m/s^2 and gyro(:,3) in rad/s
%
% Author: S. Bruder

clear;                  % Clear all variables from the workspace
close all;              % Close all windows
clc;                    % "Clean" the command window

g = 9.79221;            % Local value of gravity (m/s^2)

folder_name = uigetdir('', 'Select the Folder Containing Your Calibration Data Files:'); % Select the data folder
disp('CALIBRATION Data Folder:');
disp(folder_name)
%--------------------------------------------------------------------------
%% Calibrate the accelerometers
%--------------------------------------------------------------------------
% Load the data files and calculate averages
load([folder_name,'\z_up_accel.mat']);         % Z-Up configuration
f_z_up  = mean(accel, 1)';      % Average of the Accelerometer measurements
    
load([folder_name,'\z_down_accel.mat']);       % Z-Down configuration
f_z_down = mean(accel, 1)';     % Average of the Accelerometer measurements     
        
load([folder_name,'\y_up_accel.mat']);         % Y-Up configuration
f_y_up  = mean(accel, 1)';      % Average of the Accelerometer measurements
        
load([folder_name,'\y_down_accel.mat']);       % Y-Down configuration
f_y_down = mean(accel, 1)';     % Average of the Accelerometer measurements   
        
load([folder_name,'\x_up_accel.mat']);         % X-Up configuration
f_x_up  = mean(accel, 1)';    % Average of the Accelerometer measurements

load([folder_name,'\x_down_accel.mat']);       % X-Down configuration
f_x_down = mean(accel, 1)';    % Average of the Accelerometer measurements


% Print mean values
fprintf('z up:   %.5f   %.5f   %.5f\n', f_z_up(1), f_z_up(2), f_z_up(3))
fprintf('z down: %.5f   %.5f   %.5f\n', f_z_down(1), f_z_down(2), f_z_down(3))
fprintf('y up:   %.5f   %.5f   %.5f\n', f_y_up(1), f_y_up(2), f_y_up(3))
fprintf('y down: %.5f   %.5f   %.5f\n', f_y_down(1), f_y_down(2), f_y_down(3))
fprintf('x up:   %.5f   %.5f   %.5f\n', f_x_up(1), f_x_up(2), f_x_up(3))
fprintf('x down: %.5f   %.5f   %.5f\n', f_x_down(1), f_x_down(2), f_x_down(3))


% Calibrate the IMU using 6-side Method    
fprintf('Calibration parameters:\n')
%% Calibrate the Accels
%  Fixed Bias Calibration
b_aFB = 1/2 * [(f_x_up(1) + f_x_down(1));
               (f_y_up(2) + f_y_down(2));
               (f_z_up(3) + f_z_down(3))];

fprintf('b_aFB = [ %f , %f, %f] in m/s^2\n', b_aFB);             
             
% Scale Factor Error Terms
s_ax = 1/(2*g) * (f_x_up(1) - f_x_down(1)) - 1;
s_ay = 1/(2*g) * (f_y_up(2) - f_y_down(2)) - 1;
s_az = 1/(2*g) * (f_z_up(3) - f_z_down(3)) - 1;

% Misalignment Error Terms
m_axz = 1/(2*g) * (f_z_up(1) - f_z_down(1));
m_ayz = 1/(2*g) * (f_z_up(2) - f_z_down(2));

m_axy = 1/(2*g) * (f_y_up(1) - f_y_down(1));
m_azy = 1/(2*g) * (f_y_up(3) - f_y_down(3));

m_ayx = 1/(2*g) * (f_x_up(2) - f_x_down(2));
m_azx = 1/(2*g) * (f_x_up(3) - f_x_down(3));

% Determine the scale-factor error and misalignment matrix
Ma = [s_ax      m_axy   m_axz
      m_ayx     s_ay    m_ayz 
      m_azx     m_azy   s_az];
disp('Ma = ');
Ma
save([folder_name,'\accel_cal_params.mat'], 'b_aFB', 'Ma', 'Fs');



