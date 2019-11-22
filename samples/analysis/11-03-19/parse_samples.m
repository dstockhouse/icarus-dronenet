
clear;
clc;

% Load raw data
raw_imu_csv = readmatrix('data_imu.log');
raw_gps_csv = readmatrix('data_gpe.log');

imu.num_samples = length(raw_imu_csv);
gps.num_samples = length(raw_gps_csv);


% Time and durations, the same duration for both but different Fs
imu.Fs = 50;
imu.dt = 1/imu.Fs;

gps.Fs = 5;
gps.dt = 1/gps.Fs;

imu_duration = (imu.num_samples-1) * imu.dt;
gps_duration = (gps.num_samples-1) * gps.dt;
duration = min(imu_duration, gps_duration);
imu.duration = duration;
gps.duration = duration;

imu.t_sec = 0:imu.dt:duration;
gps.t_sec = 0:gps.dt:duration;

imu.index = 1:length(imu.t_sec);
gps.index = 1:length(gps.t_sec);


%% Get values from CSV files

imu.magneto_b = raw_imu_csv(:,2:4);
imu.magneto_b = imu.magneto_b(imu.index);

imu.f_b__i_b_tilde = raw_imu_csv(:,5:7);
imu.f_b__i_b_tilde = imu.f_b__i_b_tilde(imu.index,:)';

imu.w_b__i_b_tilde = raw_imu_csv(:,8:10);
imu.w_b__i_b_tilde = imu.w_b__i_b_tilde(imu.index,:)';

imu.temp = raw_imu_csv(:,11);
imu.temp = imu.temp(imu.index)';

imu.baro = raw_imu_csv(:,12);
imu.baro = imu.baro(imu.index)';


gps.gps_time = raw_gps_csv(:,2);
gps.gps_time = gps.gps_time(gps.index)';

gps.gps_week = raw_gps_csv(:,3);
gps.gps_week = gps.gps_week(gps.index)';

gps.gps_lock = raw_gps_csv(:,4);
gps.gps_lock = gps.gps_lock(gps.index)';

gps.gps_num_sats = raw_gps_csv(:,5);
gps.gps_num_sats = gps.gps_num_sats(gps.index)';

gps.r_e__e_b = raw_gps_csv(:,6:8);
gps.r_e__e_b = gps.r_e__e_b(gps.index,:)';

gps.v_e__e_b = raw_gps_csv(:,9:11);
gps.v_e__e_b = gps.v_e__e_b(gps.index,:)';

% GPS Accuracy
gps.delta_r_e__e_b = raw_gps_csv(:,12:14);
gps.delta_r_e__e_b = gps.delta_r_e__e_b(gps.index,:)';

gps.delta_speed = raw_gps_csv(:,15);
gps.delta_speed = gps.delta_speed(gps.index)';

gps.delta_time = raw_gps_csv(:,16);
gps.delta_time = gps.delta_time(gps.index)';



