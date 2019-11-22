
% Files were parsed in bash, now load into variables and save to .mat files
x_minus = readmatrix('calibration_xm.log');
x_plus = readmatrix('calibration_xp.log');
y_minus = readmatrix('calibration_ym.log');
y_plus = readmatrix('calibration_yp.log');
z_minus = readmatrix('calibration_zm.log');
z_plus = readmatrix('calibration_zp.log');

lengths = [length(x_minus) length(x_plus) ...
           length(y_minus) length(y_plus) ...
           length(z_minus) length(z_plus)];
       
newlength = min(lengths);

% Trim and store calibration data

Fs = 50;
SN = 'NULL'; % I don't have this stored

accel = x_plus(end-600:end,:);
save('x_up_accel.mat',   'accel', 'Fs', 'SN');

accel = x_minus(end-600:end,:);
save('x_down_accel.mat',   'accel', 'Fs', 'SN');

accel = y_plus(end-600:end,:);
save('y_up_accel.mat',   'accel', 'Fs', 'SN');

accel = y_minus(end-600:end,:);
save('y_down_accel.mat',   'accel', 'Fs', 'SN');

accel = z_plus(end-600:end,:);
save('z_up_accel.mat',   'accel', 'Fs', 'SN');

accel = z_minus(end-600:end,:);
save('z_down_accel.mat',   'accel', 'Fs', 'SN');


