function plot_IMU(w_b__i_b_tilde, f_b__i_b_tilde, constants)
%
% FUNCTION DESCRIPTION:
%   Plots IMU measurements
%
%   Adapted from Dr. Bruder's EE 440 examples
%
% INPUTS:
%   w_b__i_b_tilde  = Gyro measurements (rad/s)
%   f_b__i_b_tilde  = Accel measurements (m/s^2)
%

% Trim samples, compute duration
sample_num = min(length(w_b__i_b_tilde), length(f_b__i_b_tilde));
w_b__i_b_tilde = w_b__i_b_tilde(:,1:sample_num);
f_b__i_b_tilde = f_b__i_b_tilde(:,1:sample_num);
% samples_dur = sample_num / constants.dt;

t_sec = ((0:sample_num-1) * constants.dt);        % Simulation time vector

% Plot the angular velocity
w_b__i_b_tilde  = w_b__i_b_tilde * 180/pi;  % Convert to deg/s

figure('Units', 'normalized', 'Position', [0.01 0.01 0.4 0.8])
subplot(3,1,1)
plot(t_sec, w_b__i_b_tilde(1,:),'r', 'MarkerSize',2, 'LineWidth', 2)
ylabel('\omega_x (°/s)')
legend('Meas')
title(['Meas Angular vel'],'interpreter','tex');
subplot(3,1,2)
plot(t_sec, w_b__i_b_tilde(2,:),'g', 'MarkerSize',2, 'LineWidth', 2)
ylabel('\omega_y (°/s)')
legend('Meas')
subplot(3,1,3)
plot(t_sec, w_b__i_b_tilde(3,:),'b', 'MarkerSize',2, 'LineWidth', 2)
ylabel('\omega_z (°/s)')
legend('Meas')
xlabel('Time (sec)')

% Plot the specific force
figure('Units', 'normalized', 'Position', [0.5 0.01 0.4 0.8])
subplot(3,1,1)
plot(t_sec, f_b__i_b_tilde(1,:),'r', 'MarkerSize',2, 'LineWidth', 2)
ylabel('a_x (m/s^2)')
legend('Meas')
title(['Meas Specific Force'],'interpreter','tex');
subplot(3,1,2)
plot(t_sec, f_b__i_b_tilde(2,:),'g', 'MarkerSize',2, 'LineWidth', 2)
ylabel('a_y (m/s^2)')
legend('Meas')
subplot(3,1,3)
plot(t_sec, f_b__i_b_tilde(3,:),'b', 'MarkerSize',2, 'LineWidth', 2)
ylabel('a_z (m/s^2)')
legend('Meas')
xlabel('Time (sec)')

end
