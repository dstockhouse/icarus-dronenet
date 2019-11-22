
clear;
close all;

load('samples_11-03-2019');


% Plots
figure;
plot(imu.t_sec, imu.f_b__i_b_tilde)
title('Accel')
legend({'x', 'y', 'z'})
grid on;

figure;
plot(imu.t_sec, imu.w_b__i_b_tilde*180/pi)
legend({'x', 'y', 'z'})
title('Gyro (deg)')
grid on;

figure;
plot(imu.t_sec, imu.magneto_b)
legend({'x', 'y', 'z'})
title('Magneto')
grid on;

figure;
plot(imu.t_sec, imu.baro)
title('Baro')
grid on;

figure;
plot(imu.t_sec, imu.temp)
title('Temp')
grid on;

%% Plot all combined (dirty)
figure;
hacc = plot(imu.t_sec, imu.f_b__i_b_tilde)
hold on;
hgyr = plot(imu.t_sec, imu.w_b__i_b_tilde)
hmag = plot(imu.t_sec, imu.magneto_b)
hbar = plot(imu.t_sec, smooth(imu.baro - 83.7)*10)
htmp = plot(imu.t_sec, imu.temp)
grid on;

title('All combined');
legend({'a_x', 'a_y', 'a_z', 'w_x', 'w_y', 'w_z', 'm_x', 'm_y', 'm_z',...
    'baro', 'temp'}, 'location', 'best')

