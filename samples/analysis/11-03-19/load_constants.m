% Script/Function Description:
%   Script which defines many constant parameters

% Set to '0' for Low-fidelity and set to '1' for High-fidelity
constants.fidelity = 1;             % Decide to run high-fidelity or low-fidelity code

%------------------------------------------------------------------------------
%% Sampling rate variables and simulation time
%------------------------------------------------------------------------------
constants.Fs  = 50;                % Sample frequency (Hz)
constants.dt  = 1/constants.Fs;     % Sample interval (sec)
constants.t_start = 0;              % Simulation start time (sec)
constants.t_end = 30;               % Simulation end time (sec)

%------------------------------------------------------------------------------
%% WGS84 Earth model parameters
%------------------------------------------------------------------------------
w_ie = 72.92115167e-6;
constants.w_ie = w_ie;              % Earth's rotational rate (rad/s)
constants.w_i__i_e = [0; 0; w_ie];  % Angular velocity of {e} wrt {i} resolved in {i} (rad/s)
constants.Ohm_i__i_e = [ 0   , -w_ie, 0; ...  % Skew symmetric version of w_i__i_e (rad/s)
                         w_ie,  0   , 0; ...
                         0   ,  0   , 0];
constants.mu = 3.986004418e14;      % Earth's gravitational constant (m^3/s^2)
constants.J2 = 1.082627e-3;         % Earth's second gravitational constant
constants.R0 = 6378137.0;           % Earth's equatorial radius (meters)
constants.Rp = 6356752.314245;      % Earth's polar radius (meters)
constants.e = 0.0818191908425;      % Eccentricity
constants.f = 1 / 298.257223563;    % Flattening


constants.gravity = 9.79193;

%------------------------------------------------------------------------------
%% Starting Latitude and Longitude of Prescott Airport: Defining the Tangential frame
%------------------------------------------------------------------------------
constants.L_t = 34.6155794*pi/180;          % Latitude  of the org of t-frame (rad)
constants.lambda_t = -112.4504*pi/180;      % Longitude of the org of t-frame (rad)
constants.h_t = 1578;                       % Height    of the org of t-frame (m)

% Atitude of the Tangential frame wrt the ECEF frame
constants.C_e__t = Lat_Lon_2C_e__n(constants.L_t, constants.lambda_t);
                 
% Position of the org of {t} wrt {e} resolved in {e} (in meters)
constants.r_e__e_t = llh2xyz(constants.L_t, constants.lambda_t, constants.h_t, constants);


% %--------------------------------------------------------------------------
% % IMU error characterization values - KVH 1750 IMU
% %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% 
% rng(21498723);                      % Initialize the random number generator
% 
% %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% % VN200 Gyro model
% %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% 
% 
% % Bias terms
% %   1. Fixed Bias
% b_g_FB = 0.02*3600;                    %  Bias - Fixed Bias term (deg/hr)
% b_g_FB = b_g_FB*(pi/180)/3600;  % Convert from: (deg/hr) -> (rad/s)
% constants.gyro.b_g_FB = b_g_FB; % Bias - Fixed Bias term (rad/s)
% 
% %   2. Bias Stability
% b_g_BS = 1;                     % Bias - Bias Stability term 1-sigma: From datasheet (deg/hr)
% b_g_BS = b_g_BS*(pi/180)/3600;  % Convert from: (deg/hr) -> (rad/s)
% constants.gyro.b_g_BS = b_g_BS*randn(3,1);   % Bias - Bias Stability term (rad/s) - Random constant
% 
% %   3. Bias Instability  - DO NOT MODIFY this
% b_g_BI_sigma = 10;                          % Bias - Bias Instability term 1-sigma (deg/hr)
% b_g_BI_sigma = b_g_BI_sigma*(pi/180)/3600;  % Convert from: (deg/hr) -> (rad/s)
% constants.gyro.b_g_BI_sigma = b_g_BI_sigma; % Bias - Bias Instability Bias term 1-sigma (rad/s)
% constants.gyro.BI.correlation_time = 3;     % Correlation time for the bias instability (sec)
% constants.gyro.BI.sigma_n = sqrt((1-exp((-2*constants.dt)/constants.gyro.BI.correlation_time))) *...
%                                  constants.gyro.b_g_BI_sigma;            % Std of the BI white noise n_g (rad/s) 
% 
% % Noise terms
% gyro_PSD = .0035*3600;                 % Gyro PSD 3600*(deg/s)/rt_Hz -> PSD (deg/hr)/rt_Hz
% constants.gyro.ARW = gyro_PSD / 60;     % Gyro PSD (deg/hr)/rt_Hz / 60 -> ARW deg/rt-hr
% % Convert ARW in (deg/rt-hr) -> sigma in (rad/s)
% deg_per_rt_hr_2_rad_per_sec = (pi/180)*sqrt(constants.Fs)/60; 
% % Convert PSD in (deg/hr)/rt-Hz -> sigma in (rad/s)
% deg_per_hr_per_rt_Hz_2_rad_per_sec = (pi/180)*sqrt(constants.Fs)/3600; 
% constants.gyro.sigma_ARW = constants.gyro.ARW*deg_per_rt_hr_2_rad_per_sec; % Standard deviation of white noise due to ARW (rad/s)
% 
% % Scale factor stability & misalignment terms
% sfs = 1000;                    % Scale factor stability (ppm)
% s_g_x = sfs * 1e-6;             % x-axis scale factor error (ppm * 1e-6)
% s_g_y = sfs * 1e-6;             % y-axis scale factor error (ppm * 1e-6);
% s_g_z = sfs * 1e-6;             % z-axis scale factor error (ppm * 1e-6);
% 
% m_g_xy =  1.0e-3;               % Misalignment of y-axis into x-axis (in rad)
% m_g_xz =  0.5e-3;               % Misalignment of z-axis into x-axis
% m_g_yx = -1.0e-3;               % Misalignment of x-axis into y-axis
% m_g_yz =  0.2e-3;               % Misalignment of z-axis into y-axis
% m_g_zx = -0.5e-3;               % Misalignment of x-axis into z-axis
% m_g_zy =  0.7e-3;               % Misalignment of y-axis into z-axis
% 
% constants.gyro.M_g = ...
%    [s_g_x , m_g_xy, m_g_xz; ... % The combined Misalignment / Scale Factor matrix (dimensionless)
%     m_g_yx, s_g_y , m_g_yz; ...
%     m_g_zx, m_g_zy, s_g_z ];
%    
% g_sens = 20;                    % Gyro G-sensitivity (deg/hr)/g
% g_sens = ((g_sens*pi/180)/3600)/9.8;  % Gyro G-sensitivity (rad/sec)/(m/s^2)
% 
% constants.gyro.G_g = ...        % The gyro G-sensitivity matrix (rad/sec)/(m/s^2)
%    [g_sens , 0      , 0; ...    
%     0      , g_sens , 0; ...
%     0      , 0      , g_sens ];
% 
% fprintf('Gyro:\n')
% fprintf('  b_g,FB = %.6f (rad/s)\n  b_g,BS = %.6f (rad/s)\n  b_g,BI_sigma = %.6f (rad/s)\n  b_g,n_sigma = %.6f (rad/s)\n  ARW_sigma = %.6f (rad/s)\n  g_sensitivity = %.6f (rad/s)/(m/s^2)\n  SFS = %.6f \n\n', ...
%     constants.gyro.b_g_FB, b_g_BS, constants.gyro.b_g_BI_sigma, constants.gyro.BI.sigma_n, constants.gyro.sigma_ARW, g_sens, sfs*1e-6)
% 
% %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% % VN200 Accel specific terms
% %++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
% % Convert VRW in (m/s)/rt-hr -> sigma in (m/s^2)
% m_sec_per_rt_hr_2_m_per_sec_sq = sqrt(constants.Fs)/60; 
% % Convert PSD in mg/rt-Hz -> sigma in (m/s^2)
% mg_per_rt_Hz_2_m_per_sec_sq = sqrt(constants.Fs)/100; 
% 
% load('../vn200_calibration/accel_cal_params')
% 
% % Bias terms
% %   1. Fixed Bias
% constants.accel.b_a_FB = 0.5*9.8*1e-3;     % Bias - Fixed Bias term (m/s^2)
% 
% %   2. Bias Stability
% b_a_BS = 0.1;                     % Bias - Bias Stability term (mg)
% b_a_BS = b_a_BS * 9.8*1e-3;     % % Convert from: (mg) -> (m/s^2)
% constants.accel.b_a_BS = b_a_BS * randn(3,1);   % Bias - Bias Stability Bias term (m/s^2) - Random constant
% 
% %   3. Bias Instability
% b_a_BI_sigma = 0.04;                        % Bias - Bias Instability term 1-sigma (mg)
% b_a_BI_sigma = b_a_BI_sigma * 9.8*1e-3;     % Convert from: (mg) -> (m/s^2)
% constants.accel.b_a_BI_sigma = b_a_BI_sigma;% Bias - Bias Instability Bias term 1-sigma (m/s^2)
% constants.accel.BI.correlation_time = 2;    % Correlation time for the bias instability (sec)
% constants.accel.BI.sigma_n = sqrt((1-exp((-2*constants.dt)/constants.accel.BI.correlation_time))) * ...
%                                  constants.accel.b_a_BI_sigma; % Std of the BI white noise (m/s^2) 
% 
% % Noise terms
% accel_PSD = 0.14;               % Accel PSD (mg/rt-Hz) 
%                                 % Standard deviation of white noise due to VRW (m/s^2)
% constants.accel.sigma_VRW = accel_PSD * mg_per_rt_Hz_2_m_per_sec_sq;  
% 
% % Scale factor stability & misalignment terms
% sfs = 5000;                     % Scale factor stability (ppm)
% s_a_x = sfs * 1e-6;             % x-axis scale factor error (ppm * 1e-6)
% s_a_y = sfs * 1e-6;             % y-axis scale factor error (ppm * 1e-6);
% s_a_z = sfs * 1e-6;             % z-axis scale factor error (ppm * 1e-6);
% 
% m_a_xy = -1.0e-3;               % Misalignment of y-axis into x-axis (in rad)
% m_a_xz =  0.1e-3;               % Misalignment of z-axis into x-axis
% m_a_yx =  0.5e-3;               % Misalignment of x-axis into y-axis
% m_a_yz =  1.0e-3;               % Misalignment of z-axis into y-axis
% m_a_zx =  0.5e-3;               % Misalignment of x-axis into z-axis
% m_a_zy = -0.5e-3;               % Misalignment of y-axis into z-axis
% 
% constants.accel.M_a = ...
%    [s_a_x , m_a_xy, m_a_xz; ... % The combined Misalignment / Scale Factor matrix (dimensionless)
%     m_a_yx, s_a_y , m_a_yz; ...
%     m_a_zx, m_a_zy, s_a_z ];
% 
% fprintf('\nAccel:\n')
% fprintf('  b_a,FB =  %.6f (m/s^2)\n  b_a,BS =  %.6f (m/s^2)\n  b_a,BI_sigma =  %.6f (m/s^2)\n  b_a,n_sigma =  %.6f (m/s^2)\n  VRW_sigma =  %.6f (m/s^2)\n  SFS =  %.6f \n\n', ...
%     constants.accel.b_a_FB, b_a_BS, constants.accel.b_a_BI_sigma, constants.accel.BI.sigma_n, constants.accel.sigma_VRW, sfs*1e-6)
