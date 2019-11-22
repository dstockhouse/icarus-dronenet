
clear;
clear TAN_mech;
close all;
clc;

load_constants

% Load calibration parameters
load('../vn200_calibration/accel_cal_params')

% Load raw data
raw_csv = readmatrix('data_imu.log');

% Get values from CSV file
magneto = raw_csv(:,2:4);
f_b__i_b_tilde_uncal = raw_csv(:,5:7);
w_b__i_b_tilde_uncal = raw_csv(:,8:10);

% Not used
temp = raw_csv(:,11);
baro = raw_csv(:,12);

figure;
plot(baro)


num_samples = min(length(w_b__i_b_tilde_uncal), length(f_b__i_b_tilde_uncal));
% num_samples = 250;

w_b__i_b_tilde_uncal = w_b__i_b_tilde_uncal(1:num_samples, :)';
f_b__i_b_tilde_uncal = f_b__i_b_tilde_uncal(1:num_samples, :)';


%% Compute calibrated IMU measurements
f_b__i_b_tilde = (eye(3) + Ma)^-1*(f_b__i_b_tilde_uncal - b_aFB);

gyro_q = mean(w_b__i_b_tilde_uncal(:,1:250)');
b_gFB = gyro_q';
w_b__i_b_tilde = w_b__i_b_tilde_uncal - b_gFB; 

% Apply shady scaling by g
f_b__i_b_tilde = f_b__i_b_tilde * constants.gravity / norm(mean(f_b__i_b_tilde(:,1:250)'));

%% Plot raw IMU data
plot_IMU(w_b__i_b_tilde, f_b__i_b_tilde, constants);

%% Self-alignment
accel_q = mean(f_b__i_b_tilde(:,1:250)');
gyro_q = mean(w_b__i_b_tilde(:,1:250)');
[roll, pitch, yaw, C_n__b] = coarse_alignment_scalar(gyro_q, accel_q);

% yaw = 0;
% C_n__b = ypr2dcm(yaw, pitch, roll);

P_init = [0; 0; 0];
V_init = [0; 0; 0];
A_init = C_n__b;


% Compute average gravity here
% local_g = norm(accel_q);
% constants.gravity = 9.79193;


%% Compute mechanization

r_t__t_b = zeros(3, num_samples);
v_t__t_b = zeros(3, num_samples);
a_t__t_b = zeros(3, num_samples);
C_t__b = zeros(3, 3, num_samples);

for ii=1:length(f_b__i_b_tilde)
    
    [r_t__t_b(:, ii), v_t__t_b(:, ii), a_t__t_b(:, ii), C_t__b(:, :, ii)] = TAN_mech(w_b__i_b_tilde(:,ii), f_b__i_b_tilde(:,ii), P_init, V_init, A_init, constants);
    
end

%% Plot results
plot_PVA(r_t__t_b', v_t__t_b', a_t__t_b', C_t__b, constants);


% %% Plot track in video
% figure('Name', 'Animated track', 'units', 'normalized', 'position', [.01 .01 .9 .9]);
% hp = plot_frame_orig(C_t__b(:,:,1), r_t__t_b(:,1), '', 'k');
% hold on;
% view(135, 30)
% xlabel('x');
% ylabel('y');
% zlabel('z');
% grid on;
% xlim auto
% ylim auto
% zlim auto
% for ii=1:length(f_b__i_b_tilde) - 10
%     delete(hp);
%     hp = plot_frame_orig(C_t__b(:,:,ii), r_t__t_b(:,ii), '', 'k');
%     plot3(r_t__t_b(1,1:ii), r_t__t_b(2,1:ii), r_t__t_b(3,1:ii), 'color', [.5 .5 .5])
%     title(['t = ', string(ii*constants.dt)])
%     xlim auto
%     ylim auto
%     zlim auto
% %     lim([r_t__t_b(1,ii)-2 r_t__t_b(1,ii)+2 ...
% %             r_t__t_b(2,ii)-2 r_t__t_b(2,ii)+2 ...
% %             r_t__t_b(3,ii)-2 r_t__t_b(3,ii)+2]);
%     pause(1e-3);
% end

