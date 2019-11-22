function plot_PVA(P_est, V_est, accel_est, A_est, constants)
%
% FUNCTION DESCRIPTION:
%   Plots PVA truth. If PVA estimates/measurements are provided will also plot
%   PVA and PVA errors
%
%   P_est   = Estimated/measured position (meters)
%   V_est   = Estimated/measured velocity (meters/sec)
%   A_est   = Estimated/measured attitude/orientation as a rotation matrix
%
% OUTPUTS:

fidelity = constants.fidelity;           % Set to '0' for Low-fidelity and '1' for High-fidelity
if fidelity
    fig_label = ['High-Fidelity Tan Mechanization '];
else
    fig_label = ['Low-Fidelity Tan Mechanization '];
end

I3 = eye(3);                                % 3X3 Indentiy matrix

sample_num = min(length(P_est), length(P_est));

t_sec = ((0:sample_num-1) * constants.dt)';        % Simulation time vector
N = length(t_sec);                          % Length of the simulation time vector

%% 3D Plot of trajectory
n = 15;         % Reduce the number of points plotted by factor of 'n'
x =  P_est(1:n:end, 1);
y = -P_est(1:n:end, 2);
z = -P_est(1:n:end, 3);

figure('Name',fig_label,'Units', 'normalized', 'Position', [0.01 0.01 0.8 0.8])
h1 = stem3(x,y,z,'Marker', 'none');
title('Motion of the UAV (in a North-West-Up frame)')
view(-55, 50);
hold on;
u = gradient(x);   % Directional vector field
v = gradient(y);
w = gradient(z);
scale = 0;
quiver3(x,y,z, u, v, w, scale)
xlabel('x-dir (North in m)')
ylabel('y-dir (West in m)')
zlabel('z-dir (height in m)')

text(0, 0, 0, 'Start   End', 'HorizontalAlignment' , 'center' ,'Color', 'r')
axis equal

% x =  P_est(1:n:end, 1);
% y = -P_est(1:n:end, 2);
% z = -P_est(1:n:end, 3);
% 
% h2 = scatter3(x, y, z, 'o', 'k');

legend(h1, 'Est', 'Location', 'best')
hold off

%% Plot the position

figure('Name',fig_label,'Units', 'normalized', 'Position', [0.5 0.5 0.4 0.4])
subplot(3,1,1)
plot(t_sec, P_est(:,1),'r')
ylabel('r_x (m)')
legend('Est','Location','best')
title(['Position r^t_{tb}'],'interpreter','tex');
subplot(3,1,2)
plot(t_sec, P_est(:,2),'g')
ylabel('r_y (m)')
legend('Est','Location','best')
subplot(3,1,3)
plot(t_sec, P_est(:,3),'b')
ylabel('r_z (m)')
legend('Est','Location','best')
xlabel('Time (sec)')


%% Plot the velocity

figure('Name',fig_label,'Units', 'normalized', 'Position', [0.01 0.01 0.4 0.4])
subplot(3,1,1)
plot(t_sec, V_est(:,1),'r')
ylabel('v_x (m/s)')
legend('Est','Location','best')
title(['Velocity v^t_{tb}'],'interpreter','tex');
subplot(3,1,2)
plot(t_sec, V_est(:,2),'g')
ylabel('v_y (m/s)')
legend('Est','Location','best')
subplot(3,1,3)
plot(t_sec, V_est(:,3),'b')
ylabel('v_z (m/s)')
legend('Est','Location','best')
xlabel('Time (sec)')


%% Plot the acceleration

figure('Name',fig_label,'Units', 'normalized', 'Position', [0.01 0.01 0.4 0.4])
subplot(3,1,1)
plot(t_sec, accel_est(:,1)','r')
ylabel('a_x (m/s)')
legend('Est','Location','best')
title(['Acceleration a^t_{tb}'],'interpreter','tex');
subplot(3,1,2)
plot(t_sec, accel_est(:,2)','g')
ylabel('a_y (m/s)')
legend('Est','Location','best')
subplot(3,1,3)
plot(t_sec, accel_est(:,3),'b')
ylabel('a_z (m/s)')
legend('Est','Location','best')
xlabel('Time (sec)')

% Plot the attitude

k_vec = zeros(3,N);                         % Angle/axis representation of A_truth (i.e. C matrix)
for i=1:N
    k_vec(:,i)     = dcm2k(A_est(:,:,i)); % Convert Rotation matrix to an Angle/axis k-vector
end


figure('Name',fig_label,'Units', 'normalized', 'Position', [0.5 0.01 0.4 0.4])
subplot(3,1,1)
plot(t_sec, k_vec(1,:),'r')
ylabel('k_x')
legend('Est','Location','best')
title([' Attitude ', 'C^t_b',' as an Angle/Axis k vector'],'interpreter','tex');
subplot(3,1,2)
plot(t_sec, k_vec(2,:),'g')
ylabel('k_y')
legend('Est','Location','best')
subplot(3,1,3)
plot(t_sec, k_vec(3,:),'b')
ylabel('k_z')
legend('Est','Location','best')
xlabel('Time (sec)')

end